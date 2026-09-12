#!/usr/bin/env python3
"""ROS 2 adapter for the standalone ``phantom_chessboard`` library.

This node deliberately contains no Phantom wire-protocol implementation. BLE
discovery, command encoding, notification decoding, and high-level physical
operations live in the separately installable Python package. The node only
translates between typed library events and ROS topics/services.

Threading model
---------------
``rclpy.spin`` runs on the ROS executor thread. Bleak uses asyncio, so this node
creates a dedicated asyncio loop on the ``phantom_ble`` daemon thread. ROS
callbacks submit coroutines with ``run_coroutine_threadsafe``; library events
cross back through a thread-safe ``queue.Queue`` and are published by a short
ROS timer. Long physical operations therefore never block the ROS executor.

Service semantics
-----------------
The Trigger services for snap, recalibrate and home report that an operation was
accepted/queued. Actual mechanical completion is asynchronous and is reported
through ``/chess/phantom/status`` and the diagnostic event stream.
"""

from __future__ import annotations

import asyncio
from concurrent.futures import Future
import json
import queue
import threading
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from std_msgs.msg import (
    Bool,
    Empty,
    String,
)
from std_srvs.srv import Trigger

from phantom_chessboard import (
    STARTING_FEN,
    AckEvent,
    CleanEvent,
    CorrectionEvent,
    MoveEvent,
    PhantomBoard,
    ProtocolEvent,
    StatusEvent,
)


class PhantomBoardNode(Node):

    """Thin ROS 2 integration layer around ``PhantomBoard``.
    
    The chess manager remains authoritative for chess legality and game state;
    this node is transport/adaptation only.
    """
    def __init__(self) -> None:
        """Create ROS interfaces, cross-thread queues, and the dedicated BLE asyncio loop."""
        super().__init__(
            "phantom_board"
        )

        self.declare_parameter(
            "address",
            "",
        )
        self.declare_parameter(
            "scan_timeout",
            15.0,
        )
        self.declare_parameter(
            "auto_connect",
            True,
        )

        self._address = (
            self.get_parameter(
                "address"
            )
            .get_parameter_value()
            .string_value
        )

        self._scan_timeout = (
            self.get_parameter(
                "scan_timeout"
            )
            .get_parameter_value()
            .double_value
        )

        state_qos = QoSProfile(
            depth=1,
            reliability=(
                ReliabilityPolicy.RELIABLE
            ),
            durability=(
                DurabilityPolicy
                .TRANSIENT_LOCAL
            ),
        )

        self._connected_pub = (
            self.create_publisher(
                Bool,
                (
                    "/chess/phantom/"
                    "connected"
                ),
                state_qos,
            )
        )

        self._status_pub = (
            self.create_publisher(
                String,
                (
                    "/chess/phantom/"
                    "status"
                ),
                20,
            )
        )

        self._move_pub = (
            self.create_publisher(
                String,
                (
                    "/chess/phantom/"
                    "move"
                ),
                20,
            )
        )

        self._mismatch_pub = (
            self.create_publisher(
                String,
                (
                    "/chess/phantom/"
                    "mismatch"
                ),
                20,
            )
        )

        self._event_pub = (
            self.create_publisher(
                String,
                (
                    "/chess/phantom/"
                    "event"
                ),
                50,
            )
        )

        self.create_subscription(
            String,
            (
                "/chess/phantom/"
                "new_game"
            ),
            self._on_new_game,
            10,
        )

        self.create_subscription(
            String,
            (
                "/chess/phantom/"
                "motor_move"
            ),
            self._on_motor_move,
            20,
        )

        self.create_subscription(
            Empty,
            (
                "/chess/phantom/"
                "acknowledge_move"
            ),
            self._on_ack_move,
            20,
        )

        self.create_subscription(
            String,
            (
                "/chess/phantom/"
                "set_side"
            ),
            self._on_set_side,
            10,
        )

        self.create_subscription(
            String,
            (
                "/chess/phantom/"
                "reset_detection"
            ),
            self._on_reset_detection,
            10,
        )

        self.create_service(
            Trigger,
            (
                "/chess/phantom/"
                "snap_to_center"
            ),
            self._on_snap_service,
        )

        self.create_service(
            Trigger,
            (
                "/chess/phantom/"
                "recalibrate"
            ),
            self._on_recalibrate_service,
        )

        self.create_service(
            Trigger,
            (
                "/chess/phantom/"
                "home"
            ),
            self._on_home_service,
        )

        self._ros_events: (
            queue.Queue[
                tuple[str, Any]
            ]
        ) = queue.Queue()

        self._loop = (
            asyncio.new_event_loop()
        )
        self._board: (
            PhantomBoard | None
        ) = None
        self._event_task = None

        self._loop_thread = (
            threading.Thread(
                target=(
                    self
                    ._run_async_loop
                ),
                name="phantom_ble",
                daemon=True,
            )
        )
        self._loop_thread.start()

        self.create_timer(
            0.05,
            self._drain_ros_events,
        )

        auto_connect = (
            self.get_parameter(
                "auto_connect"
            )
            .get_parameter_value()
            .bool_value
        )

        if auto_connect:
            self._submit(
                self._connect(),
                "connect",
            )

    def _run_async_loop(
        self,
    ) -> None:
        """Run the dedicated asyncio event loop until node shutdown."""
        asyncio.set_event_loop(
            self._loop
        )
        self._loop.run_forever()

    async def _connect(
        self,
    ) -> None:
        """Construct/connect the standalone driver and start forwarding its events."""
        if (
            self._board is not None
            and self._board.connected
        ):
            return

        self._board = PhantomBoard(
            address=(
                self._address
                or None
            ),
            scan_timeout=(
                self._scan_timeout
            ),
        )

        await self._board.connect()

        self._ros_events.put(
            ("connected", True)
        )

        self._event_task = (
            asyncio.create_task(
                self._event_pump()
            )
        )

    async def _event_pump(
        self,
    ) -> None:
        """Move decoded library events from the BLE loop into the thread-safe ROS queue."""
        assert (
            self._board is not None
        )

        try:
            async for event in (
                self._board.events()
            ):
                self._ros_events.put(
                    ("event", event)
                )

        except asyncio.CancelledError:
            raise

        except Exception as exc:
            self._ros_events.put(
                (
                    "error",
                    (
                        "BLE event pump "
                        f"failed: {exc}"
                    ),
                )
            )
            self._ros_events.put(
                ("connected", False)
            )

    def _submit(
        self,
        coroutine,
        description: str,
    ) -> Future:
        """Submit an async driver operation without blocking a ROS callback.
        
        Exceptions are converted to queued error records and logged later on the
        ROS executor thread.
        """
        future = (
            asyncio
            .run_coroutine_threadsafe(
                coroutine,
                self._loop,
            )
        )

        def done_callback(
            done: Future,
        ) -> None:
            """Transfer an asynchronous operation failure to the ROS-thread queue."""
            try:
                done.result()
            except Exception as exc:
                self._ros_events.put(
                    (
                        "error",
                        (
                            f"{description} "
                            f"failed: {exc}"
                        ),
                    )
                )

        future.add_done_callback(
            done_callback
        )

        return future

    def _require_board(
        self,
    ) -> PhantomBoard:
        """Return the board instance or raise when connection initialisation has not occurred."""
        if self._board is None:
            raise RuntimeError(
                "Phantom has not "
                "connected"
            )

        return self._board

    def _drain_ros_events(
        self,
    ) -> None:
        """Drain queued BLE-thread records and publish/log them on the ROS thread."""
        while True:
            try:
                kind, value = (
                    self._ros_events
                    .get_nowait()
                )
            except queue.Empty:
                return

            if kind == "connected":
                self._connected_pub.publish(
                    Bool(
                        data=bool(value)
                    )
                )
                continue

            if kind == "error":
                self.get_logger().error(
                    str(value)
                )
                continue

            if kind == "event":
                self._publish_event(
                    value
                )

    def _publish_event(
        self,
        event,
    ) -> None:
        """Translate one typed Phantom library event into stable ROS topics.
        
        Human moves are published as coordinate-only UCI on the primary move topic;
        richer capture/protocol details are retained on the JSON diagnostic topic.
        """
        if isinstance(
            event,
            MoveEvent,
        ):
            self._move_pub.publish(
                String(
                    data=event.uci
                )
            )

            self._event_pub.publish(
                String(
                    data=json.dumps(
                        {
                            "type": (
                                "move"
                            ),
                            "notation": (
                                event
                                .notation
                            ),
                            "uci": (
                                event.uci
                            ),
                            "capture": (
                                event
                                .is_capture
                            ),
                        }
                    )
                )
            )
            return

        if isinstance(
            event,
            CorrectionEvent,
        ):
            self._mismatch_pub.publish(
                String(
                    data=(
                        event
                        .instruction
                    )
                )
            )

            self._event_pub.publish(
                String(
                    data=json.dumps(
                        {
                            "type": (
                                "mismatch"
                            ),
                            "piece": (
                                event.piece
                            ),
                            "actual": (
                                event
                                .actual_square
                            ),
                            "required": (
                                event
                                .required_square
                            ),
                        }
                    )
                )
            )
            return

        if isinstance(
            event,
            StatusEvent,
        ):
            self._status_pub.publish(
                String(
                    data=event.text
                )
            )

            self._event_pub.publish(
                String(
                    data=json.dumps(
                        {
                            "type": (
                                "status"
                            ),
                            "state": (
                                event
                                .state
                                .value
                            ),
                            "text": (
                                event.text
                            ),
                        }
                    )
                )
            )
            return

        if isinstance(
            event,
            CleanEvent,
        ):
            self._event_pub.publish(
                String(
                    data=json.dumps(
                        {
                            "type": (
                                "clean"
                            ),
                            "text": (
                                event.text
                            ),
                        }
                    )
                )
            )
            return

        if isinstance(
            event,
            AckEvent,
        ):
            self._event_pub.publish(
                String(
                    data=json.dumps(
                        {
                            "type": (
                                "ack"
                            ),
                            "code": (
                                event.code
                            ),
                        }
                    )
                )
            )
            return

        if isinstance(
            event,
            ProtocolEvent,
        ):
            self._event_pub.publish(
                String(
                    data=json.dumps(
                        {
                            "type": (
                                "protocol"
                            ),
                            "opcode": (
                                event.opcode
                            ),
                            "payload_hex": (
                                event
                                .payload
                                .hex()
                            ),
                        }
                    )
                )
            )

    def _on_new_game(
        self,
        message: String,
    ) -> None:
        """Parse a JSON new-game request and queue ``PhantomBoard.new_game``."""
        try:
            value = json.loads(
                message.data
            )

            board = (
                self._require_board()
            )

            self._submit(
                board.new_game(
                    fen=value.get(
                        "fen",
                        STARTING_FEN,
                    ),
                    human_side=(
                        value[
                            "human_side"
                        ]
                    ),
                ),
                "new game",
            )

        except Exception as exc:
            self.get_logger().error(
                "Invalid new_game "
                f"request: {exc}"
            )

    def _on_motor_move(
        self,
        message: String,
    ) -> None:
        """Queue one computer/K9-controlled physical move from a ROS string message."""
        try:
            board = (
                self._require_board()
            )

            self._submit(
                board.make_move(
                    message.data.strip()
                ),
                (
                    "motor move "
                    f"{message.data!r}"
                ),
            )

        except Exception as exc:
            self.get_logger().error(
                "Cannot command move: "
                f"{exc}"
            )

    def _on_ack_move(
        self,
        _message: Empty,
    ) -> None:
        """Acknowledge a human move after the authoritative chess layer has accepted it."""
        try:
            board = (
                self._require_board()
            )

            self._submit(
                (
                    board
                    .acknowledge_human_move()
                ),
                (
                    "acknowledge "
                    "human move"
                ),
            )

        except Exception as exc:
            self.get_logger().error(
                "Cannot acknowledge "
                f"move: {exc}"
            )

    def _on_set_side(
        self,
        message: String,
    ) -> None:
        """Set or reassert the colour controlled by the human player."""
        try:
            board = (
                self._require_board()
            )

            self._submit(
                board.set_side(
                    message.data.strip()
                ),
                "set side",
            )

        except Exception as exc:
            self.get_logger().error(
                "Cannot set side: "
                f"{exc}"
            )

    def _on_reset_detection(
        self,
        message: String,
    ) -> None:
        """Parse JSON recovery data and reconcile Phantom from the supplied FEN."""
        try:
            value = json.loads(
                message.data
            )
            board = (
                self._require_board()
            )

            self._submit(
                board.reset_detection(
                    value["fen"],
                    human_side=(
                        value.get(
                            "human_side"
                        )
                    ),
                ),
                "reset detection",
            )

        except Exception as exc:
            self.get_logger().error(
                "Invalid reset "
                "detection request: "
                f"{exc}"
            )

    def _trigger(
        self,
        operation_name: str,
        operation_factory,
        response,
    ):
        """Queue a long-running Trigger operation and return an acceptance response.
        
        Trigger cannot represent later asynchronous completion, so callers must
        monitor status/events for the final board state.
        """
        try:
            board = (
                self._require_board()
            )
            coroutine = (
                operation_factory(
                    board
                )
            )

            self._submit(
                coroutine,
                operation_name,
            )

            response.success = True
            response.message = (
                f"{operation_name} "
                "accepted; completion "
                "is reported on "
                "/chess/phantom/status"
            )

        except Exception as exc:
            response.success = False
            response.message = str(
                exc
            )

        return response

    def _on_snap_service(
        self,
        _request,
        response,
    ):
        """Queue the snap-to-centre mechanical operation."""
        return self._trigger(
            "snap to center",
            lambda board:
                board.snap_to_center(),
            response,
        )

    def _on_recalibrate_service(
        self,
        _request,
        response,
    ):
        """Queue board recalibration/homing."""
        return self._trigger(
            "recalibrate",
            lambda board:
                board.recalibrate(),
            response,
        )

    def _on_home_service(
        self,
        _request,
        response,
    ):
        """Queue return to Phantom HOME mode."""
        return self._trigger(
            "home",
            lambda board:
                board.home(),
            response,
        )

    def close(self) -> None:
        """Disconnect BLE, cancel event forwarding, stop the asyncio loop and join its thread.
        
        Shutdown errors are intentionally suppressed so teardown does not obscure
        the original reason the ROS node is exiting.
        """
        if not self._loop.is_running():
            return

        async def shutdown():
            """Cancel event forwarding and disconnect the board on the BLE loop."""
            if (
                self._event_task
                is not None
            ):
                self._event_task.cancel()

            if self._board is not None:
                await (
                    self._board
                    .disconnect()
                )

        try:
            future = (
                asyncio
                .run_coroutine_threadsafe(
                    shutdown(),
                    self._loop,
                )
            )
            future.result(
                timeout=5.0
            )
        except Exception:
            pass

        self._loop.call_soon_threadsafe(
            self._loop.stop
        )

        self._loop_thread.join(
            timeout=2.0
        )


def main(args=None) -> None:
    """ROS 2 console entry point for the Phantom board adapter."""
    rclpy.init(args=args)
    node = PhantomBoardNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.close()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
