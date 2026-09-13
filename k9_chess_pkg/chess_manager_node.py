#!/usr/bin/env python3
"""Authoritative local chess manager for K9 and the Phantom Chessboard.

This version replaces the former Lichess transport with K9's direct Phantom
BLE ROS 2 adapter while preserving the existing public K9 chess interfaces:

* ``/chess/start_game`` still starts a game for a named human player.
* ``/chess/status`` remains the durable authoritative chess status.
* ``/chess/event`` retains the event vocabulary consumed by the K9 BT.
* ``ComputeChessMove`` remains the source of K9/Stockfish moves.

Responsibility split
--------------------
``ChessManagerNode`` owns all chess rules and the authoritative
:class:`chess.Board`.  Phantom is treated as a physical input/output device:

* Phantom -> ``/chess/phantom/move`` reports a candidate human move.
* The manager validates that move with python-chess.
* Legal human moves are committed and acknowledged to Phantom.
* Illegal human moves are *not* acknowledged; the manager asks Phantom to
  reconcile back to the manager's current FEN.
* K9 moves are selected by the existing engine action server, sent to Phantom
  for physical execution, and committed to the logical board only when Phantom
  reports that the physical operation has completed.

This preserves a single source of truth: the python-chess board in this node.

Colour selection
----------------
The existing ``StartChessGame`` service contains ``player_name`` but no colour
field because the previous implementation learned colour from Lichess
``gameStart``.  To avoid changing ``k9_interfaces_pkg`` during the first direct
Phantom integration, this node uses the ``default_human_colour`` parameter.
It accepts ``WHITE`` or ``BLACK`` and defaults to ``WHITE``.

A later conversational colour-selection step can extend the service without
changing the Phantom transport design.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import time
import uuid
from typing import Optional

import chess
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger

from k9_interfaces_pkg.action import ComputeChessMove
from k9_interfaces_pkg.msg import ChessEvent, ChessStatus
from k9_interfaces_pkg.srv import (
    ControlChessGame,
    GetChessState,
    StartChessGame,
)

from .chess_text import game_started, move_instruction, piece_name


class State:
    """Externally visible chess-manager lifecycle states."""

    IDLE = "IDLE"
    STARTING = "STARTING"
    ACTIVE = "ACTIVE"
    SUSPENDED = "SUSPENDED"
    FINISHED = "FINISHED"
    ERROR = "ERROR"


@dataclass
class RuntimeState:
    """Mutable state mirrored onto ``/chess/status``."""

    state: str = State.IDLE
    player_name: str = ""
    human_colour: str = ""
    k9_colour: str = ""
    game_id: str = ""
    game_active: bool = False
    game_suspended: bool = False
    side_to_move: str = "WHITE"
    pending_move: str = ""
    last_move: str = ""
    result: str = ""
    error: str = ""
    fen: str = chess.STARTING_FEN
    ply: int = 0
    engine_busy: bool = False
    evaluation_valid: bool = False
    evaluation_pawns: float = 0.0
    evaluation_is_mate: bool = False
    mate_in: int = 0


@dataclass(frozen=True)
class MoveFacts:
    """Stable move facts used by typed ChessEvent publication."""

    uci: str
    san: str
    piece: str
    from_square: str
    to_square: str
    captured_piece: str
    gives_check: bool
    gives_mate: bool


class ChessManagerNode(Node):
    """Own local chess state and coordinate Stockfish with Phantom."""

    def __init__(self) -> None:
        super().__init__("chess_manager")

        self.declare_parameter(
            "engine_action",
            "/chess/compute_move",
        )
        self.declare_parameter(
            "engine_think_time_sec",
            2.0,
        )
        self.declare_parameter(
            "use_opening_book",
            True,
        )
        self.declare_parameter(
            "default_human_colour",
            "WHITE",
        )
        self.declare_parameter(
            "phantom_motor_timeout_sec",
            120.0,
        )
        self.declare_parameter(
            "phantom_turn_settle_sec",
            0.50,
        )
        self.declare_parameter(
            "phantom_completion_guard_sec",
            0.50,
        )

        self.engine_action_name = str(
            self.get_parameter("engine_action").value
        )
        self.engine_think_time_sec = max(
            0.05,
            float(
                self.get_parameter(
                    "engine_think_time_sec"
                ).value
            ),
        )
        self.use_opening_book = bool(
            self.get_parameter(
                "use_opening_book"
            ).value
        )

        self.default_human_colour = str(
            self.get_parameter(
                "default_human_colour"
            ).value
        ).strip().upper()

        if self.default_human_colour not in {
            "WHITE",
            "BLACK",
        }:
            self.get_logger().warning(
                "default_human_colour must be WHITE or BLACK; "
                "using WHITE"
            )
            self.default_human_colour = "WHITE"

        self.phantom_motor_timeout_sec = max(
            10.0,
            float(
                self.get_parameter(
                    "phantom_motor_timeout_sec"
                ).value
            ),
        )
        self.phantom_turn_settle_sec = max(
            0.0,
            float(
                self.get_parameter(
                    "phantom_turn_settle_sec"
                ).value
            ),
        )
        self.phantom_completion_guard_sec = max(
            0.0,
            float(
                self.get_parameter(
                    "phantom_completion_guard_sec"
                ).value
            ),
        )

        self._callback_group = ReentrantCallbackGroup()

        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ------------------------------------------------------------------
        # Existing public chess interfaces consumed by the central K9 BT.
        # ------------------------------------------------------------------

        self._status_pub = self.create_publisher(
            ChessStatus,
            "/chess/status",
            status_qos,
        )
        self._event_pub = self.create_publisher(
            ChessEvent,
            "/chess/event",
            30,
        )

        self._start_service = self.create_service(
            StartChessGame,
            "/chess/start_game",
            self._start_game_callback,
            callback_group=self._callback_group,
        )
        self._control_service = self.create_service(
            ControlChessGame,
            "/chess/control",
            self._control_callback,
            callback_group=self._callback_group,
        )
        self._state_service = self.create_service(
            GetChessState,
            "/chess/get_state",
            self._get_state_callback,
            callback_group=self._callback_group,
        )

        self._engine_client = ActionClient(
            self,
            ComputeChessMove,
            self.engine_action_name,
            callback_group=self._callback_group,
        )

        # ------------------------------------------------------------------
        # Phantom physical-board ROS interface.
        # ------------------------------------------------------------------

        self._phantom_connected_sub = self.create_subscription(
            Bool,
            "/chess/phantom/connected",
            self._phantom_connected_callback,
            status_qos,
            callback_group=self._callback_group,
        )
        self._phantom_status_sub = self.create_subscription(
            String,
            "/chess/phantom/status",
            self._phantom_status_callback,
            30,
            callback_group=self._callback_group,
        )
        self._phantom_move_sub = self.create_subscription(
            String,
            "/chess/phantom/move",
            self._phantom_move_callback,
            30,
            callback_group=self._callback_group,
        )
        self._phantom_mismatch_sub = self.create_subscription(
            String,
            "/chess/phantom/mismatch",
            self._phantom_mismatch_callback,
            30,
            callback_group=self._callback_group,
        )

        self._phantom_new_game_pub = self.create_publisher(
            String,
            "/chess/phantom/new_game",
            10,
        )
        self._phantom_motor_move_pub = self.create_publisher(
            String,
            "/chess/phantom/motor_move",
            10,
        )
        self._phantom_ack_pub = self.create_publisher(
            Empty,
            "/chess/phantom/acknowledge_move",
            10,
        )
        self._phantom_side_pub = self.create_publisher(
            String,
            "/chess/phantom/set_side",
            10,
        )
        self._phantom_reset_pub = self.create_publisher(
            String,
            "/chess/phantom/reset_detection",
            10,
        )

        self._phantom_home_client = self.create_client(
            Trigger,
            "/chess/phantom/home",
            callback_group=self._callback_group,
        )

        # ------------------------------------------------------------------
        # Authoritative local chess state.
        # ------------------------------------------------------------------

        self._runtime = RuntimeState()
        self._board = chess.Board()

        self._human_colour: Optional[chess.Color] = None
        self._k9_colour: Optional[chess.Color] = None

        self._phantom_connected = False
        self._recovering_illegal_move = False
        self._awaiting_motor_move = False
        self._motor_move_deadline = 0.0
        self._motor_move_sent_at = 0.0
        self._pending_k9_facts: Optional[MoveFacts] = None

        self._engine_goal_handle = None
        self._engine_goal_fen = ""
        self._engine_log_time = 0.0

        # Evaluation state is retained from the previous manager so the BT and
        # conversation context continue to receive the same quality of data.
        self._pending_result_eval_valid = False
        self._pending_result_eval = 0.0
        self._pending_result_is_mate = False
        self._pending_result_mate_in = 0
        self._last_post_k9_eval_valid = False
        self._last_post_k9_eval = 0.0

        self._retry_move_after = 0.0

        self._maintenance_timer = self.create_timer(
            0.20,
            self._maintenance,
            callback_group=self._callback_group,
        )
        self._status_timer = self.create_timer(
            2.0,
            self._publish_status,
            callback_group=self._callback_group,
        )

        self._publish_status()
        self.get_logger().info(
            "K9 chess manager ready; Phantom transport is direct ROS/BLE"
        )

    # ------------------------------------------------------------------
    # ROS services
    # ------------------------------------------------------------------

    def _start_game_callback(self, request, response):
        """Begin a local physical game on the directly connected Phantom.

        The service intentionally returns once board setup has been *started*.
        Physical reconciliation may take substantially longer than the BT's
        service timeout, so completion is reported asynchronously through
        ``/chess/status`` and ``/chess/event``.
        """
        player_name = str(request.player_name).strip()

        if not player_name:
            response.success = False
            response.message = "player_name must be supplied"
            return response

        if not self._phantom_connected:
            response.success = False
            response.message = "Phantom Chessboard is not connected"
            return response

        if self._runtime.game_active or self._runtime.state == State.STARTING:
            response.success = False
            response.message = "A chess game is already active or starting"
            return response

        human_name = self.default_human_colour
        human_colour = (
            chess.WHITE
            if human_name == "WHITE"
            else chess.BLACK
        )
        k9_colour = not human_colour
        k9_name = (
            "WHITE"
            if k9_colour == chess.WHITE
            else "BLACK"
        )

        self._board = chess.Board()
        self._human_colour = human_colour
        self._k9_colour = k9_colour

        self._runtime = RuntimeState(
            state=State.STARTING,
            player_name=player_name,
            human_colour=human_name,
            k9_colour=k9_name,
            game_id=f"phantom-{uuid.uuid4().hex[:8]}",
            side_to_move="WHITE",
            fen=self._board.fen(),
        )

        self._recovering_illegal_move = False
        self._awaiting_motor_move = False
        self._motor_move_deadline = 0.0
        self._motor_move_sent_at = 0.0
        self._pending_k9_facts = None
        self._clear_pending_locked()
        self._last_post_k9_eval_valid = False

        payload = {
            "fen": self._board.fen(),
            "human_side": human_name.lower(),
        }

        self._phantom_new_game_pub.publish(
            String(
                data=json.dumps(
                    payload,
                    separators=(",", ":"),
                )
            )
        )

        self._publish_event(
            "GAME_SETUP_STARTED",
            player_name=player_name,
            colour=k9_name,
            message=(
                f"Preparing Phantom board; human={human_name}, "
                f"K9={k9_name}"
            ),
        )
        self._publish_status()

        response.success = True
        response.message = (
            f"Phantom setup started; {player_name} is {human_name}, "
            f"K9 is {k9_name}"
        )
        return response

    def _control_callback(self, request, response):
        """Handle generic chess lifecycle commands."""
        command = str(request.command).strip().upper()

        if command == "SUSPEND":
            if not self._runtime.game_active:
                response.success = False
                response.message = "No active chess game"
                return response

            self._runtime.game_suspended = True
            self._runtime.state = State.SUSPENDED
            self._cancel_engine_goal()
            self._publish_event("GAME_SUSPENDED")
            self._publish_status()

            response.success = True
            response.message = "Chess game suspended"
            return response

        if command == "RESUME":
            if not self._runtime.game_active:
                response.success = False
                response.message = "No active chess game"
                return response

            self._runtime.game_suspended = False
            self._runtime.state = State.ACTIVE
            self._publish_event("GAME_RESUMED")
            self._publish_status()

            response.success = True
            response.message = "Chess game resumed"
            return response

        if command == "RESIGN":
            if not self._runtime.game_active:
                response.success = False
                response.message = "No active chess game"
                return response

            winner = self._runtime.human_colour
            self._finish_game(
                status="resign",
                winner=winner,
            )
            response.success = True
            response.message = "K9 resigned"
            return response

        if command == "ABORT":
            if not (
                self._runtime.game_active
                or self._runtime.state == State.STARTING
            ):
                response.success = False
                response.message = "No active/starting chess game"
                return response

            self._abort_game("aborted")
            response.success = True
            response.message = "Chess game aborted"
            return response

        if command == "RESET":
            if self._runtime.game_active:
                response.success = False
                response.message = "Cannot reset while a game is active"
                return response

            self._reset_local_state()
            self._request_phantom_home()
            self._publish_event("GAME_RESET")
            self._publish_status()

            response.success = True
            response.message = "Chess state reset"
            return response

        response.success = False
        response.message = (
            "command must be SUSPEND, RESUME, RESIGN, ABORT or RESET"
        )
        return response

    def _get_state_callback(self, _request, response):
        """Return the current durable ChessStatus message."""
        response.status = self._status_message()
        return response

    # ------------------------------------------------------------------
    # Phantom callbacks
    # ------------------------------------------------------------------

    def _phantom_connected_callback(self, msg: Bool) -> None:
        """Track BLE-board availability."""
        connected = bool(msg.data)
        previously_connected = self._phantom_connected
        self._phantom_connected = connected

        if connected and not previously_connected:
            self._publish_event(
                "PHANTOM_CONNECTED",
                message="Phantom Chessboard connected",
            )
            return

        if not connected and previously_connected:
            self._publish_event(
                "PHANTOM_DISCONNECTED",
                message="Phantom Chessboard disconnected",
            )

            if self._runtime.game_active:
                self._runtime.game_suspended = True
                self._runtime.state = State.SUSPENDED
                self._runtime.error = "Phantom Chessboard disconnected"
                self._cancel_engine_goal()
                self._publish_status()

    def _phantom_status_callback(self, msg: String) -> None:
        """React to high-level physical-board state transitions."""
        status = str(msg.data).strip()

        if not status:
            return

        # Motor completion has priority because "Board Playing" after a K9
        # motor command is the acknowledgement that physical state reached the
        # selected move.
        if (
            status in {"Board Playing", "BLE Playing"}
            and self._awaiting_motor_move
        ):
            # Phantom emits "BLE Playing" as part of accepting the preceding
            # human move, before the computer motor command is issued.  An
            # opening-book engine result can arrive quickly enough that this
            # residual status races with the newly armed K9 move.  A physical
            # motor move cannot plausibly complete inside this short guard
            # interval, so ignore such an early playing status and wait for the
            # post-motion one.
            elapsed = (
                time.monotonic()
                - self._motor_move_sent_at
                if self._motor_move_sent_at > 0.0
                else 0.0
            )

            if elapsed < self.phantom_completion_guard_sec:
                self.get_logger().debug(
                    "Ignoring early Phantom playing status "
                    f"{status!r} {elapsed:.3f}s after K9 move publication"
                )
                return

            self._confirm_physical_k9_move()
            return

        if (
            status in {"Board Playing", "BLE Playing"}
            and self._recovering_illegal_move
        ):
            self._recovering_illegal_move = False
            self._publish_event(
                "ILLEGAL_MOVE_RECOVERED",
                message="Phantom restored the authoritative board position",
            )
            self._publish_status()
            return

        if (
            status in {"Board Playing", "BLE Playing"}
            and self._runtime.state == State.STARTING
            and not self._runtime.game_active
        ):
            self._activate_started_game()
            return

        if status in {
            "Managing Mismatch",
            "Managing Mismatch...",
        }:
            self._publish_event(
                "BOARD_RECONCILING",
                message=status,
            )

    def _phantom_mismatch_callback(self, msg: String) -> None:
        """Surface Phantom's physical correction instructions."""
        instruction = str(msg.data).strip()

        if not instruction:
            return

        self._publish_event(
            "BOARD_CORRECTION",
            message=instruction,
        )

    def _phantom_move_callback(self, msg: String) -> None:
        """Validate and process a candidate physical move from the human."""
        uci = str(msg.data).strip().lower()

        if not self._runtime.game_active:
            self.get_logger().warning(
                f"Ignoring Phantom move outside active game: {uci!r}"
            )
            return

        if self._runtime.game_suspended:
            self.get_logger().warning(
                f"Ignoring Phantom move while suspended: {uci!r}"
            )
            return

        if self._recovering_illegal_move:
            self.get_logger().warning(
                f"Ignoring Phantom move while recovering board: {uci!r}"
            )
            return

        if self._awaiting_motor_move:
            self.get_logger().warning(
                f"Ignoring Phantom move while K9 motor move is pending: {uci!r}"
            )
            return

        if self._human_colour is None or self._board.turn != self._human_colour:
            self.get_logger().warning(
                f"Ignoring out-of-turn human move {uci!r}"
            )
            self._recover_authoritative_position(
                reason="It is not the human player's turn",
                attempted_uci=uci,
            )
            return

        try:
            move = self._resolve_human_move(uci)
        except ValueError as exc:
            self._recover_authoritative_position(
                reason=str(exc),
                attempted_uci=uci,
            )
            return

        if move not in self._board.legal_moves:
            self._recover_authoritative_position(
                reason="That move is not legal in the current position",
                attempted_uci=uci,
            )
            return

        facts = self._describe_move(
            self._board,
            move,
        )

        self._board.push(move)
        self._sync_runtime_from_board(
            last_move=facts.uci
        )

        # The official application acknowledges the accepted human move and
        # then reasserts the human side before the computer reply.
        self._phantom_ack_pub.publish(
            Empty()
        )
        self._phantom_side_pub.publish(
            String(
                data=self._runtime.human_colour.lower()
            )
        )

        # The official application waits for the side-selection transaction
        # (including Phantom's 0x06/0x04 acknowledgement) before sending the
        # computer move.  Opening-book moves can return essentially
        # immediately, so give the BLE adapter a small window to complete that
        # handshake before an engine request is allowed.
        self._retry_move_after = max(
            self._retry_move_after,
            time.monotonic()
            + self.phantom_turn_settle_sec,
        )

        self._runtime.evaluation_valid = False
        self._runtime.evaluation_is_mate = False
        self._runtime.mate_in = 0

        self._publish_move_event(
            "HUMAN_MOVE",
            facts,
        )
        self._publish_status()

        if self._finish_if_board_terminal():
            return

        # The periodic maintenance callback will now see that it is K9's turn
        # and request a move from the existing engine action server.

    def _resolve_human_move(self, uci: str) -> chess.Move:
        """Convert a Phantom coordinate move into a python-chess move.

        Phantom promotion semantics have not yet been captured.  For now, if a
        four-character pawn move reaches the back rank and a queen promotion is
        legal, queen promotion is inferred.  This mirrors common electronic
        board behaviour while keeping the special case explicit.
        """
        if len(uci) not in {4, 5}:
            raise ValueError(
                f"Unsupported Phantom move format: {uci!r}"
            )

        try:
            move = chess.Move.from_uci(uci)
        except ValueError as exc:
            raise ValueError(
                f"Invalid Phantom move {uci!r}"
            ) from exc

        if len(uci) == 4:
            piece = self._board.piece_at(
                move.from_square
            )

            if (
                piece is not None
                and piece.piece_type == chess.PAWN
                and chess.square_rank(move.to_square)
                in {0, 7}
            ):
                queen_promotion = chess.Move(
                    move.from_square,
                    move.to_square,
                    promotion=chess.QUEEN,
                )

                if queen_promotion in self._board.legal_moves:
                    return queen_promotion

        return move

    def _recover_authoritative_position(
        self,
        *,
        reason: str,
        attempted_uci: str,
    ) -> None:
        """Reject a physical move and ask Phantom to restore current FEN."""
        self._recovering_illegal_move = True

        self._publish_event(
            "ILLEGAL_HUMAN_MOVE",
            uci=attempted_uci,
            speech_hint=(
                "Negative. That move is not legal. "
                "I shall restore the board."
            ),
            message=reason,
        )

        payload = {
            "fen": self._board.fen(),
            "human_side": self._runtime.human_colour.lower(),
        }

        self._phantom_reset_pub.publish(
            String(
                data=json.dumps(
                    payload,
                    separators=(",", ":"),
                )
            )
        )

        self.get_logger().info(
            f"Rejected human move {attempted_uci}; "
            f"resetting Phantom to FEN {self._board.fen()}"
        )

    # ------------------------------------------------------------------
    # Game activation / local board updates
    # ------------------------------------------------------------------

    def _activate_started_game(self) -> None:
        """Promote STARTING to ACTIVE after Phantom setup completes."""
        self._runtime.state = State.ACTIVE
        self._runtime.game_active = True
        self._runtime.game_suspended = False
        self._runtime.error = ""

        hint = game_started(
            self._runtime.human_colour,
            self._runtime.k9_colour,
        )

        self._publish_event(
            "GAME_STARTED",
            game_id=self._runtime.game_id,
            player_name=self._runtime.player_name,
            colour=self._runtime.k9_colour,
            speech_hint=hint,
        )
        self._publish_status()

        # If K9 is White, maintenance will immediately ask the engine for the
        # opening move.  If the human is White, the board simply waits.

    def _sync_runtime_from_board(
        self,
        *,
        last_move: str = "",
    ) -> None:
        """Mirror authoritative python-chess state into RuntimeState."""
        self._runtime.fen = self._board.fen()
        self._runtime.ply = self._board.ply()
        self._runtime.side_to_move = (
            "WHITE"
            if self._board.turn == chess.WHITE
            else "BLACK"
        )

        if last_move:
            self._runtime.last_move = last_move

    @staticmethod
    def _describe_move(
        board: chess.Board,
        move: chess.Move,
    ) -> MoveFacts:
        """Describe *move* before it is pushed onto *board*."""
        moving_piece = board.piece_at(
            move.from_square
        )

        if moving_piece is None:
            raise ValueError(
                f"No piece at "
                f"{chess.square_name(move.from_square)}"
            )

        captured = board.piece_at(
            move.to_square
        )

        if board.is_en_passant(move):
            captured = chess.Piece(
                chess.PAWN,
                not moving_piece.color,
            )

        san = board.san(move)
        probe = board.copy(stack=True)
        probe.push(move)

        return MoveFacts(
            uci=move.uci(),
            san=san,
            piece=piece_name(
                moving_piece.piece_type
            ),
            from_square=chess.square_name(
                move.from_square
            ),
            to_square=chess.square_name(
                move.to_square
            ),
            captured_piece=(
                piece_name(captured.piece_type)
                if captured is not None
                else ""
            ),
            gives_check=probe.is_check(),
            gives_mate=probe.is_checkmate(),
        )

    # ------------------------------------------------------------------
    # Engine action client
    # ------------------------------------------------------------------

    def _maintenance(self) -> None:
        """Drive engine requests and detect stalled motor operations."""
        if (
            self._awaiting_motor_move
            and self._motor_move_deadline > 0.0
            and time.monotonic() >= self._motor_move_deadline
        ):
            self._handle_motor_timeout()
            return

        self._maybe_request_k9_move()

    def _maybe_request_k9_move(self) -> None:
        """Request a Stockfish move when the authoritative board says K9 turns."""
        if (
            not self._runtime.game_active
            or self._runtime.game_suspended
            or self._runtime.engine_busy
            or bool(self._runtime.pending_move)
            or self._awaiting_motor_move
            or self._recovering_illegal_move
            or self._k9_colour is None
            or self._board.turn != self._k9_colour
            or self._board.is_game_over(claim_draw=False)
            or time.monotonic() < self._retry_move_after
        ):
            return

        fen = self._board.fen()
        k9_is_white = self._k9_colour == chess.WHITE

        if not self._engine_client.server_is_ready():
            now = time.monotonic()

            if now - self._engine_log_time >= 5.0:
                self.get_logger().warning(
                    f"Waiting for chess engine action "
                    f"{self.engine_action_name}"
                )
                self._engine_log_time = now

            return

        goal = ComputeChessMove.Goal()
        goal.fen = fen
        goal.k9_is_white = k9_is_white
        goal.think_time_sec = float(
            self.engine_think_time_sec
        )
        goal.use_book = bool(
            self.use_opening_book
        )

        # Re-check the position immediately before arming the asynchronous
        # request.  ROS callbacks may have changed board state since the first
        # guard above.
        if (
            self._runtime.engine_busy
            or self._runtime.pending_move
            or self._board.fen() != fen
        ):
            return

        self._runtime.engine_busy = True
        self._engine_goal_fen = fen

        self._publish_event(
            "ENGINE_THINKING"
        )
        self._publish_status()

        future = self._engine_client.send_goal_async(
            goal,
            feedback_callback=self._engine_feedback,
        )
        future.add_done_callback(
            self._engine_goal_response
        )

    def _engine_feedback(self, feedback_msg) -> None:
        """Log sparse diagnostic engine feedback without event flooding."""
        feedback = feedback_msg.feedback

        if feedback.depth and feedback.depth % 5 == 0:
            self.get_logger().debug(
                f"Chess engine depth={feedback.depth} "
                f"nps={feedback.nodes_per_second}"
            )

    def _engine_goal_response(self, future) -> None:
        """Receive engine action goal acceptance."""
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._engine_failed(
                f"Unable to send engine goal: {exc}"
            )
            return

        if not goal_handle.accepted:
            self._engine_failed(
                "Chess engine rejected the goal"
            )
            return

        self._engine_goal_handle = goal_handle

        result_future = (
            goal_handle.get_result_async()
        )
        result_future.add_done_callback(
            self._engine_result
        )

    def _engine_result(self, future) -> None:
        """Validate selected move and hand it to the physical Phantom board."""
        try:
            wrapped = future.result()
            result = wrapped.result
        except Exception as exc:
            self._engine_failed(
                f"Chess engine result failed: {exc}"
            )
            return

        expected_fen = self._engine_goal_fen
        current_fen = self._board.fen()

        self._runtime.engine_busy = False
        self._engine_goal_handle = None
        self._engine_goal_fen = ""

        if not result.success:
            if result.error and result.error != "cancelled":
                self._engine_failed(result.error)
            else:
                self._publish_status()
            return

        if (
            not self._runtime.game_active
            or self._runtime.game_suspended
            or self._k9_colour is None
            or expected_fen != current_fen
        ):
            self._publish_event(
                "ENGINE_RESULT_DISCARDED",
                message=(
                    "Board/game state changed while engine was thinking"
                ),
            )
            self._publish_status()
            return

        try:
            board = chess.Board(current_fen)
            move = chess.Move.from_uci(
                result.best_move_uci
            )

            if move not in board.legal_moves:
                raise ValueError(
                    "Engine returned illegal move "
                    f"{result.best_move_uci}"
                )

            facts = self._describe_move(
                board,
                move,
            )
        except Exception as exc:
            self._engine_failed(str(exc))
            return

        previous_eval_valid = (
            self._last_post_k9_eval_valid
        )
        previous_eval = (
            self._last_post_k9_eval
        )
        delta_valid = (
            previous_eval_valid
            and result.position_evaluation_valid
        )
        delta = (
            float(result.position_eval_pawns)
            - previous_eval
            if delta_valid
            else 0.0
        )

        self._publish_event(
            "POSITION_EVALUATED",
            evaluation_before_valid=(
                previous_eval_valid
            ),
            evaluation_before_pawns=(
                previous_eval
            ),
            evaluation_after_valid=(
                result.position_evaluation_valid
            ),
            evaluation_after_pawns=float(
                result.position_eval_pawns
            ),
            evaluation_delta_pawns=delta,
            is_mate=bool(
                result.position_is_mate
            ),
            mate_in=int(
                result.position_mate_in
            ),
            message=(
                "Evaluation change caused by the human's previous move"
                if delta_valid
                else (
                    "Current position has a forced mate"
                    if result.position_is_mate
                    else "Current position evaluated"
                )
            ),
        )

        self._runtime.evaluation_valid = bool(
            result.position_evaluation_valid
        )
        self._runtime.evaluation_pawns = float(
            result.position_eval_pawns
        )
        self._runtime.evaluation_is_mate = bool(
            result.position_is_mate
        )
        self._runtime.mate_in = int(
            result.position_mate_in
        )

        self._runtime.pending_move = facts.uci
        self._pending_k9_facts = facts

        self._pending_result_eval_valid = bool(
            result.resulting_evaluation_valid
        )
        self._pending_result_eval = float(
            result.resulting_eval_pawns
        )
        self._pending_result_is_mate = bool(
            result.resulting_is_mate
        )
        self._pending_result_mate_in = int(
            result.resulting_mate_in
        )

        hint = move_instruction(
            facts.piece,
            facts.from_square,
            facts.to_square,
            facts.captured_piece,
            facts.gives_check,
            facts.gives_mate,
        )

        self._publish_move_event(
            "K9_MOVE_SELECTED",
            facts,
            source=str(result.source),
            evaluation_before_valid=(
                result.position_evaluation_valid
            ),
            evaluation_before_pawns=float(
                result.position_eval_pawns
            ),
            evaluation_after_valid=(
                result.resulting_evaluation_valid
            ),
            evaluation_after_pawns=float(
                result.resulting_eval_pawns
            ),
            evaluation_delta_pawns=(
                float(result.resulting_eval_pawns)
                - float(result.position_eval_pawns)
                if (
                    result.position_evaluation_valid
                    and result.resulting_evaluation_valid
                )
                else 0.0
            ),
            is_mate=result.resulting_is_mate,
            mate_in=int(
                result.resulting_mate_in
            ),
            speech_hint=hint,
        )
        self._publish_status()

        separator = (
            "x"
            if board.is_capture(move)
            else "-"
        )
        phantom_move = (
            f"{facts.from_square}"
            f"{separator}"
            f"{facts.to_square}"
        )

        self._awaiting_motor_move = True
        self._motor_move_sent_at = time.monotonic()
        self._motor_move_deadline = (
            self._motor_move_sent_at
            + self.phantom_motor_timeout_sec
        )

        # K9_MOVE_SENT retains the semantic used by the old manager: the move
        # has been handed to the transport.  K9_MOVE_CONFIRMED is emitted only
        # after the physical board returns to Board Playing.
        self._publish_event(
            "K9_MOVE_SENT",
            game_id=self._runtime.game_id,
            uci=facts.uci,
        )

        self._phantom_motor_move_pub.publish(
            String(data=phantom_move)
        )

    def _confirm_physical_k9_move(self) -> None:
        """Commit pending K9 move after Phantom reports physical completion."""
        facts = self._pending_k9_facts
        pending_uci = self._runtime.pending_move

        self._awaiting_motor_move = False
        self._motor_move_deadline = 0.0
        self._motor_move_sent_at = 0.0
        self._pending_k9_facts = None

        if not pending_uci or facts is None:
            self._set_runtime_error(
                "Phantom motor completion arrived without a pending K9 move",
                fatal=False,
            )
            return

        try:
            move = chess.Move.from_uci(
                pending_uci
            )
        except ValueError as exc:
            self._set_runtime_error(
                f"Invalid pending K9 move {pending_uci}: {exc}",
                fatal=False,
            )
            return

        if move not in self._board.legal_moves:
            self._set_runtime_error(
                f"Pending K9 move {pending_uci} is no longer legal",
                fatal=True,
            )
            return

        self._board.push(move)
        self._runtime.pending_move = ""
        self._sync_runtime_from_board(
            last_move=facts.uci
        )
        self._commit_pending_evaluation_locked()

        self._publish_move_event(
            "K9_MOVE_CONFIRMED",
            facts,
        )
        self._publish_status()

        if self._finish_if_board_terminal():
            return

        self._publish_event(
            "YOUR_MOVE",
            speech_hint="Your turn to move.",
        )

    def _handle_motor_timeout(self) -> None:
        """Fail safely when Phantom never reports completion of a motor move."""
        pending = self._runtime.pending_move

        self._awaiting_motor_move = False
        self._motor_move_deadline = 0.0
        self._motor_move_sent_at = 0.0
        self._pending_k9_facts = None
        self._runtime.pending_move = ""
        self._clear_pending_evaluation_locked()
        self._runtime.error = (
            f"Phantom timed out executing K9 move {pending}"
        )
        self._retry_move_after = (
            time.monotonic() + 2.0
        )

        self._publish_event(
            "PHANTOM_MOVE_TIMEOUT",
            uci=pending,
            message=self._runtime.error,
        )
        self._publish_status()

        # Logical board was deliberately not advanced, so reset/reconciliation
        # to the authoritative current FEN is safe.
        if self._runtime.human_colour:
            self._phantom_reset_pub.publish(
                String(
                    data=json.dumps(
                        {
                            "fen": self._board.fen(),
                            "human_side": (
                                self._runtime
                                .human_colour
                                .lower()
                            ),
                        },
                        separators=(",", ":"),
                    )
                )
            )

    def _engine_failed(self, message: str) -> None:
        """Record a recoverable engine error and allow a later retry."""
        self._runtime.engine_busy = False
        self._engine_goal_handle = None
        self._engine_goal_fen = ""
        self._runtime.error = message
        self._retry_move_after = (
            time.monotonic() + 2.0
        )

        self._publish_event(
            "ENGINE_ERROR",
            message=message,
        )
        self._publish_status()

    def _cancel_engine_goal(self) -> None:
        """Request cancellation of an accepted engine goal, if any."""
        goal_handle = self._engine_goal_handle

        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass

    # ------------------------------------------------------------------
    # End-of-game handling
    # ------------------------------------------------------------------

    def _finish_if_board_terminal(self) -> bool:
        """Finish the game if the authoritative board is terminal."""
        if not self._board.is_game_over(
            claim_draw=False
        ):
            return False

        outcome = self._board.outcome(
            claim_draw=False
        )

        if outcome is None:
            return False

        if outcome.winner is None:
            winner = ""
        else:
            winner = (
                "WHITE"
                if outcome.winner == chess.WHITE
                else "BLACK"
            )

        status = self._termination_name(
            outcome.termination
        )
        self._finish_game(
            status=status,
            winner=winner,
        )
        return True

    @staticmethod
    def _termination_name(
        termination: chess.Termination,
    ) -> str:
        """Map python-chess termination names to legacy K9 result vocabulary."""
        mapping = {
            chess.Termination.CHECKMATE: "mate",
            chess.Termination.STALEMATE: "stalemate",
            chess.Termination.INSUFFICIENT_MATERIAL: "insufficientMaterial",
            chess.Termination.SEVENTYFIVE_MOVES: "seventyfiveMoves",
            chess.Termination.FIVEFOLD_REPETITION: "fivefoldRepetition",
            chess.Termination.FIFTY_MOVES: "fiftyMoves",
            chess.Termination.THREEFOLD_REPETITION: "threefoldRepetition",
        }

        return mapping.get(
            termination,
            termination.name.lower(),
        )

    def _finish_game(
        self,
        *,
        status: str,
        winner: str = "",
    ) -> None:
        """Publish terminal state while preserving the BT's existing format."""
        if not (
            self._runtime.game_active
            or self._runtime.state == State.STARTING
        ):
            return

        winner_name = str(winner).upper()
        status = str(status or "finished")

        if winner_name in {"WHITE", "BLACK"}:
            result_text = (
                f"{winner_name}:{status}"
            )
        else:
            result_text = (
                f"DRAW:{status}"
                if status not in {"aborted", "finished"}
                else status.upper()
            )

        self._runtime.state = State.FINISHED
        self._runtime.game_active = False
        self._runtime.game_suspended = False
        self._runtime.engine_busy = False
        self._runtime.pending_move = ""
        self._runtime.result = result_text
        self._clear_pending_evaluation_locked()

        self._cancel_engine_goal()

        if winner_name and winner_name == self._runtime.k9_colour:
            hint = "The game is over. I have won."
        elif winner_name:
            hint = "Congratulations. You have won."
        else:
            hint = "The game is drawn."

        self._publish_event(
            "GAME_FINISHED",
            game_id=self._runtime.game_id,
            player_name=self._runtime.player_name,
            status=status,
            colour=winner_name,
            speech_hint=hint,
            message=result_text,
        )
        self._publish_status()

    def _abort_game(self, status: str) -> None:
        """Abort setup/game without declaring either player the winner."""
        self._cancel_engine_goal()

        self._runtime.state = State.FINISHED
        self._runtime.game_active = False
        self._runtime.game_suspended = False
        self._runtime.engine_busy = False
        self._runtime.pending_move = ""
        self._runtime.result = status.upper()

        self._awaiting_motor_move = False
        self._recovering_illegal_move = False
        self._pending_k9_facts = None
        self._clear_pending_evaluation_locked()

        self._publish_event(
            "GAME_FINISHED",
            status=status,
            message=status.upper(),
            speech_hint="The chess game has been aborted.",
        )
        self._publish_status()

    def _reset_local_state(self) -> None:
        """Return the local logical subsystem to its pristine idle state."""
        self._cancel_engine_goal()

        self._runtime = RuntimeState()
        self._board = chess.Board()
        self._human_colour = None
        self._k9_colour = None

        self._awaiting_motor_move = False
        self._recovering_illegal_move = False
        self._motor_move_deadline = 0.0
        self._motor_move_sent_at = 0.0
        self._pending_k9_facts = None

        self._clear_pending_locked()
        self._last_post_k9_eval_valid = False

    def _request_phantom_home(self) -> None:
        """Ask Phantom to return HOME when the service is available."""
        if self._phantom_home_client.service_is_ready():
            self._phantom_home_client.call_async(
                Trigger.Request()
            )

    def _set_runtime_error(
        self,
        message: str,
        *,
        fatal: bool,
    ) -> None:
        """Publish an error, optionally ending the active game."""
        self._runtime.error = message

        if fatal:
            self._runtime.state = State.ERROR
            self._runtime.game_active = False

        self._publish_event(
            "CHESS_ERROR",
            message=message,
        )
        self._publish_status()

    # ------------------------------------------------------------------
    # Message construction
    # ------------------------------------------------------------------

    def _status_message(self) -> ChessStatus:
        """Construct a complete ChessStatus snapshot."""
        state = RuntimeState(
            **vars(self._runtime)
        )

        msg = ChessStatus()
        msg.state = state.state
        msg.player_name = state.player_name
        msg.human_colour = state.human_colour
        msg.k9_colour = state.k9_colour
        msg.game_id = state.game_id
        msg.game_active = state.game_active
        msg.game_suspended = state.game_suspended
        msg.side_to_move = state.side_to_move
        msg.pending_move = state.pending_move
        msg.last_move = state.last_move
        msg.result = state.result
        msg.error = state.error
        msg.fen = state.fen
        msg.ply = int(state.ply)
        msg.engine_busy = state.engine_busy
        msg.evaluation_valid = (
            state.evaluation_valid
        )
        msg.evaluation_pawns = float(
            state.evaluation_pawns
        )
        msg.evaluation_is_mate = (
            state.evaluation_is_mate
        )
        msg.mate_in = int(
            state.mate_in
        )
        return msg

    def _publish_status(self) -> None:
        """Publish current durable status."""
        self._status_pub.publish(
            self._status_message()
        )

    def _publish_event(
        self,
        event_type: str,
        *,
        game_id: Optional[str] = None,
        player_name: Optional[str] = None,
        status: str = "",
        colour: str = "",
        uci: str = "",
        san: str = "",
        piece: str = "",
        from_square: str = "",
        to_square: str = "",
        captured_piece: str = "",
        gives_check: bool = False,
        gives_mate: bool = False,
        source: str = "",
        evaluation_before_valid: bool = False,
        evaluation_before_pawns: float = 0.0,
        evaluation_after_valid: bool = False,
        evaluation_after_pawns: float = 0.0,
        evaluation_delta_pawns: float = 0.0,
        is_mate: bool = False,
        mate_in: int = 0,
        speech_hint: str = "",
        message: str = "",
    ) -> None:
        """Publish one typed ChessEvent using the existing K9 interface."""
        msg = ChessEvent()
        msg.stamp = (
            self.get_clock().now().to_msg()
        )
        msg.type = str(event_type)
        msg.game_id = (
            self._runtime.game_id
            if game_id is None
            else str(game_id)
        )
        msg.player_name = (
            self._runtime.player_name
            if player_name is None
            else str(player_name)
        )
        msg.status = str(status)
        msg.colour = str(colour)
        msg.uci = str(uci)
        msg.san = str(san)
        msg.piece = str(piece)
        msg.from_square = str(
            from_square
        )
        msg.to_square = str(
            to_square
        )
        msg.captured_piece = str(
            captured_piece
        )
        msg.gives_check = bool(
            gives_check
        )
        msg.gives_mate = bool(
            gives_mate
        )
        msg.source = str(source)
        msg.evaluation_before_valid = bool(
            evaluation_before_valid
        )
        msg.evaluation_before_pawns = float(
            evaluation_before_pawns
        )
        msg.evaluation_after_valid = bool(
            evaluation_after_valid
        )
        msg.evaluation_after_pawns = float(
            evaluation_after_pawns
        )
        msg.evaluation_delta_pawns = float(
            evaluation_delta_pawns
        )
        msg.is_mate = bool(is_mate)
        msg.mate_in = int(mate_in)
        msg.speech_hint = str(
            speech_hint
        )
        msg.message = str(message)

        self._event_pub.publish(msg)

    def _publish_move_event(
        self,
        event_type: str,
        facts: MoveFacts,
        **extra,
    ) -> None:
        """Publish a ChessEvent populated from MoveFacts."""
        self._publish_event(
            event_type,
            uci=facts.uci,
            san=facts.san,
            piece=facts.piece,
            from_square=(
                facts.from_square
            ),
            to_square=facts.to_square,
            captured_piece=(
                facts.captured_piece
            ),
            gives_check=facts.gives_check,
            gives_mate=facts.gives_mate,
            **extra,
        )

    # ------------------------------------------------------------------
    # Evaluation / pending helpers
    # ------------------------------------------------------------------

    def _commit_pending_evaluation_locked(self) -> None:
        """Promote predicted post-K9 evaluation after physical confirmation."""
        self._runtime.evaluation_valid = (
            self._pending_result_eval_valid
        )
        self._runtime.evaluation_pawns = (
            self._pending_result_eval
        )
        self._runtime.evaluation_is_mate = (
            self._pending_result_is_mate
        )
        self._runtime.mate_in = (
            self._pending_result_mate_in
        )

        self._last_post_k9_eval_valid = (
            self._pending_result_eval_valid
        )
        self._last_post_k9_eval = (
            self._pending_result_eval
        )

        self._clear_pending_evaluation_locked()

    def _clear_pending_evaluation_locked(self) -> None:
        """Clear engine evaluation associated with an unconfirmed move."""
        self._pending_result_eval_valid = False
        self._pending_result_eval = 0.0
        self._pending_result_is_mate = False
        self._pending_result_mate_in = 0

    def _clear_pending_locked(self) -> None:
        """Clear all engine/pending-move state."""
        self._runtime.pending_move = ""
        self._runtime.engine_busy = False
        self._engine_goal_handle = None
        self._engine_goal_fen = ""
        self._clear_pending_evaluation_locked()

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def destroy_node(self):
        """Cancel engine activity and release ROS action resources."""
        self._cancel_engine_goal()

        try:
            self._engine_client.destroy()
        except Exception:
            pass

        return super().destroy_node()


def main(args=None) -> None:
    """ROS 2 console entry point."""
    rclpy.init(args=args)
    node = ChessManagerNode()

    executor = MultiThreadedExecutor(
        num_threads=4
    )
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
