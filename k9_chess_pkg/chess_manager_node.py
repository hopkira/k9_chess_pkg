#!/usr/bin/env python3
"""Authoritative Lichess/Phantom game manager for K9 chess.

The manager owns the Lichess session and authoritative python-chess board.  It
never controls K9's voice, tail, eyes, ears or back panel directly.  Instead it
publishes typed ChessStatus/ChessEvent messages for the central K9 behaviour
tree and uses the separate ComputeChessMove action server for move selection.
"""

from __future__ import annotations

from dataclasses import dataclass
import os
from queue import Empty, Queue
import threading
import time
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

from k9_interfaces_pkg.action import ComputeChessMove
from k9_interfaces_pkg.msg import ChessEvent, ChessStatus
from k9_interfaces_pkg.srv import (
    ControlChessGame,
    GetChessState,
    StartChessGame,
)

from .chess_text import game_started, move_instruction, piece_name
from .lichess_api import (
    LichessAPI,
    LichessRateLimitError,
)


class State:
    IDLE = "IDLE"
    WAITING_FOR_CHALLENGE = "WAITING_FOR_CHALLENGE"
    STARTING = "STARTING"
    ACTIVE = "ACTIVE"
    SUSPENDED = "SUSPENDED"
    FINISHED = "FINISHED"
    ERROR = "ERROR"


TERMINAL_LICHESS_STATES = {
    "aborted",
    "mate",
    "resign",
    "stalemate",
    "timeout",
    "draw",
    "outoftime",
    "cheat",
    "noStart",
    "unknownFinish",
    "variantEnd",
}


@dataclass
class RuntimeState:
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
    uci: str
    san: str
    piece: str
    from_square: str
    to_square: str
    captured_piece: str
    gives_check: bool
    gives_mate: bool


class ChessManagerNode(Node):
    """Own Lichess/Phantom game lifecycle and local board synchronisation."""

    def __init__(self) -> None:
        super().__init__("chess_manager")

        self.declare_parameter("lichess_url", "https://lichess.org/api/")
        self.declare_parameter(
            "phantom_player_username",
            os.getenv("LICHESS_USERNAME", "hopkira"),
        )
        self.declare_parameter("unexpected_challenge_decline_reason", "generic")
        self.declare_parameter("engine_action", "/chess/compute_move")
        self.declare_parameter("engine_think_time_sec", 2.0)
        self.declare_parameter("use_opening_book", True)
        self.declare_parameter("mirror_move_instruction_to_chat", False)
        self.declare_parameter("stream_reconnect_max_sec", 30.0)

        self.lichess_url = str(
            self.get_parameter("lichess_url").value
        )
        self.phantom_player_username = str(
            self.get_parameter("phantom_player_username").value
        ).strip()
        self.unexpected_challenge_decline_reason = str(
            self.get_parameter("unexpected_challenge_decline_reason").value
        ).strip() or "generic"
        self.engine_action_name = str(
            self.get_parameter("engine_action").value
        )
        self.engine_think_time_sec = max(
            0.05,
            float(self.get_parameter("engine_think_time_sec").value),
        )
        self.use_opening_book = bool(
            self.get_parameter("use_opening_book").value
        )
        self.mirror_move_instruction_to_chat = bool(
            self.get_parameter("mirror_move_instruction_to_chat").value
        )
        self.stream_reconnect_max_sec = max(
            1.0,
            float(self.get_parameter("stream_reconnect_max_sec").value),
        )

        bot_token = os.getenv("LICHESS_BOT_TOKEN", "").strip()

        self._bot_api: Optional[LichessAPI] = None

        if bot_token:
            self._bot_api = LichessAPI(bot_token, self.lichess_url)

        self._callback_group = ReentrantCallbackGroup()

        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

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

        self._lock = threading.RLock()
        self._runtime = RuntimeState()
        self._board = chess.Board()
        self._known_moves: list[str] = []
        self._k9_colour: Optional[chess.Color] = None

        self._engine_goal_handle = None
        self._engine_goal_fen = ""
        self._engine_log_time = 0.0

        self._pending_result_eval_valid = False
        self._pending_result_eval = 0.0
        self._pending_result_is_mate = False
        self._pending_result_mate_in = 0
        self._last_post_k9_eval_valid = False
        self._last_post_k9_eval = 0.0

        self._retry_move_after = 0.0

        self._network_queue: Queue = Queue()
        self._shutdown_event = threading.Event()
        self._event_response = None
        self._game_response = None
        self._game_stream_game_id = ""
        self._game_thread: Optional[threading.Thread] = None
        self._pending_challenge_id = ""

        self._drain_timer = self.create_timer(
            0.05,
            self._drain_network_queue,
            callback_group=self._callback_group,
        )
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

        if self._bot_api is None:
            self._runtime.state = State.ERROR
            self._runtime.error = "LICHESS_BOT_TOKEN is not set"
            self.get_logger().error(self._runtime.error)
        else:
            threading.Thread(
                target=self._event_stream_worker,
                name="k9-lichess-events",
                daemon=True,
            ).start()

        self._publish_status()
        self.get_logger().info(
            "K9 chess manager ready; Phantom transport is via Lichess"
        )

    # ------------------------------------------------------------------
    # ROS services
    # ------------------------------------------------------------------

    def _start_game_callback(self, request, response):
        """Arm K9 to accept the next Phantom challenge from hopkira.

        The Phantom phone application creates the Lichess game.  This service
        therefore records the real human player's display name and puts the
        chess subsystem into WAITING_FOR_CHALLENGE; it does not create a
        Lichess challenge itself.
        """
        player_name = str(request.player_name).strip()
        if not player_name:
            response.success = False
            response.message = "player_name must be supplied"
            return response

        with self._lock:
            if self._runtime.game_active or self._runtime.state in {
                State.WAITING_FOR_CHALLENGE,
                State.STARTING,
            }:
                response.success = False
                response.message = "A chess game is already active or expected"
                return response

        if self._bot_api is None:
            response.success = False
            response.message = "LICHESS_BOT_TOKEN is not set"
            return response

        if not self.phantom_player_username:
            response.success = False
            response.message = "phantom_player_username is not configured"
            return response

        with self._lock:
            self._runtime = RuntimeState(
                state=State.WAITING_FOR_CHALLENGE,
                player_name=player_name,
            )
            self._board = chess.Board()
            self._known_moves = []
            self._k9_colour = None
            self._pending_challenge_id = ""
            self._clear_pending_locked()
            self._last_post_k9_eval_valid = False

        speech_hint = (
            "Please start the game against me in the Phantom application."
        )
        self._publish_event(
            "WAITING_FOR_CHALLENGE",
            player_name=player_name,
            speech_hint=speech_hint,
            message=(
                "Waiting for Lichess challenge from "
                f"{self.phantom_player_username}"
            ),
        )
        self._publish_status()

        response.success = True
        response.message = (
            "Chess ready; waiting for the Phantom/Lichess challenge from "
            f"{self.phantom_player_username}"
        )
        return response

    def _control_callback(self, request, response):
        command = str(request.command).strip().upper()

        if command == "SUSPEND":
            with self._lock:
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
            with self._lock:
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

        if command in {"RESIGN", "ABORT"}:
            with self._lock:
                game_id = self._runtime.game_id
                active = self._runtime.game_active
            if not active or not game_id or self._bot_api is None:
                response.success = False
                response.message = "No active Lichess game"
                return response

            try:
                if command == "RESIGN":
                    self._bot_api.resign(game_id)
                else:
                    self._bot_api.abort(game_id)
                self._publish_event(f"{command}_REQUESTED")
                response.success = True
                response.message = f"{command.lower()} requested"
            except Exception as exc:
                response.success = False
                response.message = str(exc)
            return response

        if command == "RESET":
            with self._lock:
                if self._runtime.game_active:
                    response.success = False
                    response.message = "Cannot reset while a game is active"
                    return response
                self._runtime = RuntimeState()
                self._board = chess.Board()
                self._known_moves = []
                self._k9_colour = None
                self._pending_challenge_id = ""
                self._clear_pending_locked()
                self._last_post_k9_eval_valid = False
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
        response.status = self._status_message()
        return response

    # ------------------------------------------------------------------
    # Lichess streaming workers
    # ------------------------------------------------------------------

    def _event_stream_worker(self) -> None:
        delay = 1.0

        while not self._shutdown_event.is_set():
            try:
                response = self._bot_api.event_stream()
                self._event_response = response
                delay = 1.0

                for event in LichessAPI.iter_json_lines(response):
                    if self._shutdown_event.is_set():
                        return
                    self._network_queue.put(("LOBBY", event))

                self._network_queue.put(
                    ("STREAM_NOTICE", "Lobby event stream ended; reconnecting")
                )

            except LichessRateLimitError as exc:
                self._network_queue.put(("STREAM_NOTICE", str(exc)))
                if self._shutdown_event.wait(60.0):
                    return
            except Exception as exc:
                self._network_queue.put(
                    (
                        "STREAM_NOTICE",
                        f"Lobby stream error: {exc}; reconnecting in {delay:.1f}s",
                    )
                )
                if self._shutdown_event.wait(delay):
                    return
                delay = min(delay * 2.0, self.stream_reconnect_max_sec)
            finally:
                try:
                    if self._event_response is not None:
                        self._event_response.close()
                except Exception:
                    pass
                self._event_response = None

    def _start_game_stream(self, game_id: str) -> None:
        with self._lock:
            if (
                self._game_thread is not None
                and self._game_thread.is_alive()
                and self._game_stream_game_id == game_id
            ):
                return
            self._game_stream_game_id = game_id

        self._game_thread = threading.Thread(
            target=self._game_stream_worker,
            args=(game_id,),
            name=f"k9-lichess-game-{game_id}",
            daemon=True,
        )
        self._game_thread.start()

    def _game_stream_worker(self, game_id: str) -> None:
        delay = 1.0

        while not self._shutdown_event.is_set():
            with self._lock:
                if (
                    not self._runtime.game_active
                    or self._runtime.game_id != game_id
                ):
                    return

            try:
                response = self._bot_api.game_stream(game_id)
                self._game_response = response
                delay = 1.0

                for event in LichessAPI.iter_json_lines(response):
                    if self._shutdown_event.is_set():
                        return
                    self._network_queue.put(("GAME", game_id, event))

                with self._lock:
                    still_active = (
                        self._runtime.game_active
                        and self._runtime.game_id == game_id
                    )
                if still_active:
                    self._network_queue.put(
                        (
                            "STREAM_NOTICE",
                            f"Game stream {game_id} ended; reconnecting",
                        )
                    )

            except LichessRateLimitError as exc:
                self._network_queue.put(("STREAM_NOTICE", str(exc)))
                if self._shutdown_event.wait(60.0):
                    return
            except Exception as exc:
                self._network_queue.put(
                    (
                        "STREAM_NOTICE",
                        f"Game stream error: {exc}; reconnecting in {delay:.1f}s",
                    )
                )
                if self._shutdown_event.wait(delay):
                    return
                delay = min(delay * 2.0, self.stream_reconnect_max_sec)
            finally:
                try:
                    if self._game_response is not None:
                        self._game_response.close()
                except Exception:
                    pass
                self._game_response = None

    # ------------------------------------------------------------------
    # Network event handling on ROS executor thread
    # ------------------------------------------------------------------

    def _drain_network_queue(self) -> None:
        for _ in range(100):
            try:
                item = self._network_queue.get_nowait()
            except Empty:
                return

            kind = item[0]
            if kind == "LOBBY":
                self._handle_lobby_event(item[1])
            elif kind == "GAME":
                self._handle_game_event(item[1], item[2])
            elif kind == "MOVE_SENT":
                self._handle_move_sent(item[1], item[2])
            elif kind == "MOVE_SEND_ERROR":
                self._handle_move_send_error(item[1], item[2], item[3])
            elif kind == "CHALLENGE_ACCEPTED":
                self._handle_challenge_accepted(*item[1:])
            elif kind == "CHALLENGE_ACCEPT_ERROR":
                self._handle_challenge_accept_error(*item[1:])
            elif kind == "STREAM_NOTICE":
                self.get_logger().warning(str(item[1]))

    def _handle_lobby_event(self, event: dict) -> None:
        event_type = str(event.get("type", ""))

        if event_type == "challenge":
            challenge = event.get("challenge", {}) or {}
            challenge_id = str(challenge.get("id", ""))

            # Lichess exposes both incoming and outgoing challenges on the bot
            # event stream.  V2.1 never creates an outgoing challenge, but
            # ignore one defensively if it appears.
            direction = str(challenge.get("direction", "")).lower()
            if direction == "out":
                self._publish_event(
                    "OUTGOING_CHALLENGE_IGNORED",
                    game_id=challenge_id,
                )
                return

            challenger = challenge.get("challenger", {}) or {}
            challenger_username = str(
                challenger.get("name")
                or challenger.get("id")
                or ""
            ).strip()

            expected = self.phantom_player_username.strip()
            expected_match = (
                bool(challenger_username)
                and challenger_username.casefold() == expected.casefold()
            )

            with self._lock:
                current_state = self._runtime.state
                active = self._runtime.game_active
                display_name = self._runtime.player_name

            # All physical K9 games are the fixed hopkira -> k9_bot Lichess
            # route.  Never accept a challenge from another account.
            if not expected_match:
                threading.Thread(
                    target=self._decline_worker,
                    args=(challenge_id,),
                    daemon=True,
                ).start()
                self._publish_event(
                    "CHALLENGE_DECLINED",
                    player_name=challenger_username,
                    message=(
                        f"Unexpected challenger; expected {expected}"
                    ),
                )
                return

            # The central K9 BT must explicitly arm chess first.  This prevents
            # a stale or accidental Phantom challenge from starting a game
            # while K9 is doing something else.
            if (
                active
                or current_state
                not in {State.WAITING_FOR_CHALLENGE}
            ):
                threading.Thread(
                    target=self._decline_worker,
                    args=(challenge_id,),
                    daemon=True,
                ).start()
                self._publish_event(
                    "CHALLENGE_DECLINED",
                    player_name=display_name,
                    message=(
                        "Already playing"
                        if active
                        else "K9 is not waiting for a chess challenge"
                    ),
                )
                return

            with self._lock:
                self._runtime.state = State.STARTING
                self._runtime.error = ""
                self._pending_challenge_id = challenge_id

            self._publish_event(
                "CHALLENGE_ACCEPTING",
                game_id=challenge_id,
                player_name=display_name,
                message=(
                    f"Accepting Phantom challenge from {challenger_username}"
                ),
            )
            self._publish_status()

            threading.Thread(
                target=self._accept_challenge_worker,
                args=(challenge_id,),
                daemon=True,
            ).start()
            return

        if event_type == "gameStart":
            game = event.get("game", {}) or {}
            game_id = str(game.get("id", ""))
            colour = str(game.get("color", "")).upper()

            with self._lock:
                pending = self._pending_challenge_id
                active = self._runtime.game_active
                current_game = self._runtime.game_id
                display_name = self._runtime.player_name

            if not game_id:
                return

            # Only activate the game K9 has just accepted, or refresh an
            # already-active matching game after stream reconnection.
            if (
                (pending and game_id == pending)
                or (active and game_id == current_game)
            ):
                self._activate_game(
                    game_id,
                    colour,
                    display_name,
                )
            else:
                self._publish_event(
                    "GAME_START_IGNORED",
                    game_id=game_id,
                    message="Game was not the expected Phantom challenge",
                )
            return

        if event_type == "gameFinish":
            game = event.get("game", {}) or {}
            game_id = str(game.get("id", ""))
            with self._lock:
                current = self._runtime.game_id
                active = self._runtime.game_active
            if game_id and game_id == current and active:
                self._finish_game(
                    status=str(game.get("status", "finished")),
                    winner=str(game.get("winner", "")),
                )
            return

        if event_type in {"challengeCanceled", "challengeDeclined"}:
            challenge = event.get("challenge", {}) or {}
            challenge_id = str(challenge.get("id", ""))

            with self._lock:
                if (
                    self._runtime.state == State.STARTING
                    and (
                        not challenge_id
                        or challenge_id == self._pending_challenge_id
                    )
                ):
                    self._runtime.state = State.WAITING_FOR_CHALLENGE
                    self._runtime.error = event_type
                    self._pending_challenge_id = ""

            self._publish_event(
                event_type.upper(),
                game_id=challenge_id,
                player_name=self._runtime.player_name,
                speech_hint=(
                    "Please start the game against me again in the "
                    "Phantom application."
                ),
            )
            self._publish_status()

    def _decline_worker(self, challenge_id: str) -> None:
        try:
            self._bot_api.decline_challenge(
                challenge_id,
                self.unexpected_challenge_decline_reason,
            )
        except Exception as exc:
            self.get_logger().warning(
                f"Unable to decline challenge {challenge_id}: {exc}"
            )

    def _accept_challenge_worker(
        self,
        challenge_id: str,
    ) -> None:
        try:
            self._bot_api.accept_challenge(challenge_id)
            self._network_queue.put(
                ("CHALLENGE_ACCEPTED", challenge_id)
            )
        except Exception as exc:
            self._network_queue.put(
                (
                    "CHALLENGE_ACCEPT_ERROR",
                    challenge_id,
                    str(exc),
                )
            )

    def _handle_challenge_accepted(
        self,
        challenge_id: str,
    ) -> None:
        with self._lock:
            display_name = self._runtime.player_name

        self._publish_event(
            "CHALLENGE_ACCEPTED",
            game_id=challenge_id,
            player_name=display_name,
        )
        # gameStart supplies K9's assigned colour.  The challenge and game ids
        # are the same; wait for gameStart before activating the board.

    def _handle_challenge_accept_error(
        self,
        challenge_id: str,
        error: str,
    ) -> None:
        with self._lock:
            if (
                self._runtime.state == State.STARTING
                and (
                    not self._pending_challenge_id
                    or challenge_id == self._pending_challenge_id
                )
            ):
                self._runtime.state = State.WAITING_FOR_CHALLENGE
                self._runtime.error = error
                self._pending_challenge_id = ""

            display_name = self._runtime.player_name

        self._publish_event(
            "CHALLENGE_ACCEPT_FAILED",
            game_id=challenge_id,
            player_name=display_name,
            message=error,
            speech_hint=(
                "I could not accept that game. Please try starting it again "
                "in the Phantom application."
            ),
        )
        self._publish_status()

    def _handle_game_event(self, game_id: str, event: dict) -> None:
        with self._lock:
            if game_id != self._runtime.game_id:
                return

        event_type = str(event.get("type", ""))
        if event_type == "gameFull":
            state = event.get("state", {}) or {}
        elif event_type == "gameState":
            state = event
        else:
            return

        self._process_game_state(state)

    # ------------------------------------------------------------------
    # Board state and move events
    # ------------------------------------------------------------------

    def _process_game_state(self, state: dict) -> None:
        move_text = str(state.get("moves", "")).strip()
        moves = move_text.split() if move_text else []
        status = str(state.get("status", "started"))
        winner = str(state.get("winner", ""))

        board = chess.Board()
        move_records: list[tuple[chess.Color, MoveFacts]] = []

        with self._lock:
            previous_moves = list(self._known_moves)
            k9_colour = self._k9_colour
            pending_move = self._runtime.pending_move

        common = 0
        max_common = min(len(previous_moves), len(moves))
        while common < max_common and previous_moves[common] == moves[common]:
            common += 1

        if common < len(previous_moves):
            self._publish_event(
                "BOARD_RESYNC",
                message="Lichess move history changed; rebuilding local board",
            )

        try:
            for index, uci in enumerate(moves):
                move = chess.Move.from_uci(uci)
                if move not in board.legal_moves:
                    raise ValueError(
                        f"Illegal Lichess move {uci} at {board.fen()}"
                    )

                mover = board.turn
                facts = self._describe_move(board, move)
                board.push(move)

                if index >= common:
                    move_records.append((mover, facts))

        except Exception as exc:
            self._set_runtime_error(
                f"Unable to reconstruct Lichess game: {exc}",
                fatal=False,
            )
            return

        with self._lock:
            self._board = board
            self._known_moves = moves
            self._runtime.fen = board.fen()
            self._runtime.ply = len(moves)
            self._runtime.last_move = moves[-1] if moves else ""
            self._runtime.side_to_move = "WHITE" if board.turn else "BLACK"

        for mover, facts in move_records:
            if k9_colour is not None and mover == k9_colour:
                event_name = "K9_MOVE_CONFIRMED"
                mismatch = ""
                with self._lock:
                    if pending_move:
                        if facts.uci == pending_move:
                            self._commit_pending_evaluation_locked()
                            self._runtime.pending_move = ""
                            self._retry_move_after = 0.0
                        else:
                            mismatch = (
                                f"Lichess confirmed K9 move {facts.uci} while "
                                f"{pending_move} was pending"
                            )
                            self._runtime.pending_move = ""
                            self._clear_pending_evaluation_locked()
                            self._runtime.error = mismatch
                self._publish_move_event(event_name, facts)
                if mismatch:
                    self._publish_event("BOARD_RESYNC", message=mismatch)
            elif k9_colour is not None:
                with self._lock:
                    # The authoritative board has moved since K9's previous
                    # post-move evaluation; current-position evaluation is now
                    # unknown until the engine evaluates this position.
                    self._runtime.evaluation_valid = False
                    self._runtime.evaluation_is_mate = False
                    self._runtime.mate_in = 0
                self._publish_move_event("HUMAN_MOVE", facts)
            else:
                self._publish_move_event("MOVE", facts)

        if status in TERMINAL_LICHESS_STATES:
            self._finish_game(status=status, winner=winner)
            return

        with self._lock:
            if self._runtime.game_active:
                self._runtime.state = (
                    State.SUSPENDED
                    if self._runtime.game_suspended
                    else State.ACTIVE
                )

        self._publish_status()

        if (
            move_records
            and k9_colour is not None
            and board.turn != k9_colour
            and not board.is_game_over(claim_draw=False)
        ):
            # K9's move has reached Lichess/Phantom; this is the earliest safe
            # point for the BT to prompt the human to move.
            last_mover, _last_facts = move_records[-1]
            if last_mover == k9_colour:
                self._publish_event(
                    "YOUR_MOVE",
                    speech_hint="Your move.",
                )

    @staticmethod
    def _describe_move(board: chess.Board, move: chess.Move) -> MoveFacts:
        moving_piece = board.piece_at(move.from_square)
        if moving_piece is None:
            raise ValueError(f"No piece at {chess.square_name(move.from_square)}")

        captured = board.piece_at(move.to_square)
        if board.is_en_passant(move):
            captured = chess.Piece(chess.PAWN, not moving_piece.color)

        san = board.san(move)
        probe = board.copy(stack=True)
        probe.push(move)

        return MoveFacts(
            uci=move.uci(),
            san=san,
            piece=piece_name(moving_piece.piece_type),
            from_square=chess.square_name(move.from_square),
            to_square=chess.square_name(move.to_square),
            captured_piece=(
                piece_name(captured.piece_type) if captured is not None else ""
            ),
            gives_check=probe.is_check(),
            gives_mate=probe.is_checkmate(),
        )

    # ------------------------------------------------------------------
    # Engine action client
    # ------------------------------------------------------------------

    def _maintenance(self) -> None:
        self._maybe_request_k9_move()

    def _maybe_request_k9_move(self) -> None:
        with self._lock:
            if (
                not self._runtime.game_active
                or self._runtime.game_suspended
                or self._runtime.engine_busy
                or bool(self._runtime.pending_move)
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
                    f"Waiting for chess engine action {self.engine_action_name}"
                )
                self._engine_log_time = now
            return

        goal = ComputeChessMove.Goal()
        goal.fen = fen
        goal.k9_is_white = k9_is_white
        goal.think_time_sec = float(self.engine_think_time_sec)
        goal.use_book = bool(self.use_opening_book)

        with self._lock:
            # Re-check after leaving the lock for server readiness.
            if (
                self._runtime.engine_busy
                or self._runtime.pending_move
                or self._board.fen() != fen
            ):
                return
            self._runtime.engine_busy = True
            self._engine_goal_fen = fen

        self._publish_event("ENGINE_THINKING")
        self._publish_status()

        future = self._engine_client.send_goal_async(
            goal,
            feedback_callback=self._engine_feedback,
        )
        future.add_done_callback(self._engine_goal_response)

    def _engine_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        # Feedback is diagnostic only; avoid flooding ChessEvent at engine
        # iteration frequency.  The final result is published structurally.
        if feedback.depth and feedback.depth % 5 == 0:
            self.get_logger().debug(
                f"Chess engine depth={feedback.depth} nps={feedback.nodes_per_second}"
            )

    def _engine_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._engine_failed(f"Unable to send engine goal: {exc}")
            return

        if not goal_handle.accepted:
            self._engine_failed("Chess engine rejected the goal")
            return

        with self._lock:
            self._engine_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._engine_result)

    def _engine_result(self, future) -> None:
        try:
            wrapped = future.result()
            result = wrapped.result
        except Exception as exc:
            self._engine_failed(f"Chess engine result failed: {exc}")
            return

        with self._lock:
            expected_fen = self._engine_goal_fen
            current_fen = self._board.fen()
            active = self._runtime.game_active
            suspended = self._runtime.game_suspended
            game_id = self._runtime.game_id
            k9_colour = self._k9_colour
            previous_eval_valid = self._last_post_k9_eval_valid
            previous_eval = self._last_post_k9_eval
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
            not active
            or suspended
            or not game_id
            or k9_colour is None
            or expected_fen != current_fen
        ):
            self._publish_event(
                "ENGINE_RESULT_DISCARDED",
                message="Board/game state changed while engine was thinking",
            )
            self._publish_status()
            return

        try:
            board = chess.Board(current_fen)
            move = chess.Move.from_uci(result.best_move_uci)
            if move not in board.legal_moves:
                raise ValueError(
                    f"Engine returned illegal move {result.best_move_uci}"
                )
            facts = self._describe_move(board, move)
        except Exception as exc:
            self._engine_failed(str(exc))
            return

        delta_valid = (
            previous_eval_valid and result.position_evaluation_valid
        )
        delta = (
            float(result.position_eval_pawns) - previous_eval
            if delta_valid
            else 0.0
        )

        self._publish_event(
            "POSITION_EVALUATED",
            evaluation_before_valid=previous_eval_valid,
            evaluation_before_pawns=previous_eval,
            evaluation_after_valid=result.position_evaluation_valid,
            evaluation_after_pawns=float(result.position_eval_pawns),
            evaluation_delta_pawns=delta,
            message=(
                "Evaluation change caused by the human's previous move"
                if delta_valid
                else "Current position evaluated"
            ),
        )

        with self._lock:
            # Status evaluation now describes the current position, before K9
            # physically/virtually makes the selected move.
            self._runtime.evaluation_valid = bool(
                result.position_evaluation_valid
            )
            self._runtime.evaluation_pawns = float(
                result.position_eval_pawns
            )
            self._runtime.evaluation_is_mate = False
            self._runtime.mate_in = 0

            self._runtime.pending_move = facts.uci
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
            evaluation_before_valid=result.position_evaluation_valid,
            evaluation_before_pawns=float(result.position_eval_pawns),
            evaluation_after_valid=result.resulting_evaluation_valid,
            evaluation_after_pawns=float(result.resulting_eval_pawns),
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
            mate_in=int(result.resulting_mate_in),
            speech_hint=hint,
        )
        self._publish_status()

        threading.Thread(
            target=self._send_move_worker,
            args=(game_id, facts.uci, hint),
            name=f"k9-send-move-{facts.uci}",
            daemon=True,
        ).start()

    def _engine_failed(self, message: str) -> None:
        with self._lock:
            self._runtime.engine_busy = False
            self._engine_goal_handle = None
            self._engine_goal_fen = ""
            self._runtime.error = message
            self._retry_move_after = time.monotonic() + 2.0
        self._publish_event("ENGINE_ERROR", message=message)
        self._publish_status()

    def _cancel_engine_goal(self) -> None:
        with self._lock:
            goal_handle = self._engine_goal_handle
            # Even if send_goal_async has not yet returned a goal handle, a
            # stale result will be discarded because suspension/FEN is checked.
        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Move submission
    # ------------------------------------------------------------------

    def _send_move_worker(
        self,
        game_id: str,
        move_uci: str,
        speech_hint: str,
    ) -> None:
        try:
            self._bot_api.make_move(game_id, move_uci)
            if self.mirror_move_instruction_to_chat and speech_hint:
                try:
                    self._bot_api.chat(game_id, speech_hint)
                except Exception as exc:
                    self._network_queue.put(
                        ("STREAM_NOTICE", f"Lichess chat failed: {exc}")
                    )
            self._network_queue.put(("MOVE_SENT", game_id, move_uci))
        except Exception as exc:
            self._network_queue.put(
                ("MOVE_SEND_ERROR", game_id, move_uci, str(exc))
            )

    def _handle_move_sent(self, game_id: str, move_uci: str) -> None:
        with self._lock:
            if game_id != self._runtime.game_id:
                return
        self._publish_event(
            "K9_MOVE_SENT",
            game_id=game_id,
            uci=move_uci,
        )

    def _handle_move_send_error(
        self,
        game_id: str,
        move_uci: str,
        error: str,
    ) -> None:
        with self._lock:
            if game_id != self._runtime.game_id:
                return
            if self._runtime.pending_move == move_uci:
                self._runtime.pending_move = ""
                self._clear_pending_evaluation_locked()
            self._runtime.error = error
            self._retry_move_after = time.monotonic() + 2.0
        self._publish_event(
            "MOVE_SEND_ERROR",
            game_id=game_id,
            uci=move_uci,
            message=error,
        )
        self._publish_status()

    # ------------------------------------------------------------------
    # Game activation/finish
    # ------------------------------------------------------------------

    def _activate_game(
        self,
        game_id: str,
        k9_colour_name: str,
        player_name: str = "",
    ) -> None:
        k9_colour_name = str(k9_colour_name).upper()
        if k9_colour_name not in {"WHITE", "BLACK"}:
            # A reconnect may omit colour; preserve an already-known value.
            with self._lock:
                k9_colour_name = self._runtime.k9_colour

        if k9_colour_name not in {"WHITE", "BLACK"}:
            self._set_runtime_error(
                f"Unable to determine K9 colour for game {game_id}",
                fatal=True,
            )
            return

        new_game = False
        with self._lock:
            if self._runtime.game_active:
                if self._runtime.game_id == game_id:
                    self._runtime.k9_colour = k9_colour_name
                    self._k9_colour = (
                        chess.WHITE
                        if k9_colour_name == "WHITE"
                        else chess.BLACK
                    )
                    return
                self.get_logger().warning(
                    f"Ignoring gameStart {game_id}; already playing "
                    f"{self._runtime.game_id}"
                )
                return

            if player_name:
                self._runtime.player_name = player_name
            human_colour_name = (
                "BLACK" if k9_colour_name == "WHITE" else "WHITE"
            )
            self._runtime.human_colour = human_colour_name
            self._runtime.k9_colour = k9_colour_name
            self._runtime.game_id = game_id
            self._runtime.game_active = True
            self._runtime.game_suspended = False
            self._runtime.state = State.ACTIVE
            self._runtime.side_to_move = "WHITE"
            self._runtime.fen = chess.STARTING_FEN
            self._runtime.ply = 0
            self._runtime.pending_move = ""
            self._runtime.last_move = ""
            self._runtime.result = ""
            self._runtime.error = ""
            self._runtime.engine_busy = False
            self._runtime.evaluation_valid = False
            self._runtime.evaluation_is_mate = False
            self._runtime.mate_in = 0

            self._board = chess.Board()
            self._known_moves = []
            self._k9_colour = (
                chess.WHITE if k9_colour_name == "WHITE" else chess.BLACK
            )
            self._clear_pending_locked()
            self._last_post_k9_eval_valid = False
            self._pending_challenge_id = ""
            new_game = True

        if not new_game:
            return

        human_colour_name = self._runtime.human_colour
        hint = game_started(human_colour_name, k9_colour_name)

        self._publish_event(
            "GAME_STARTED",
            game_id=game_id,
            player_name=self._runtime.player_name,
            colour=k9_colour_name,
            speech_hint=hint,
        )
        self._publish_status()
        self._start_game_stream(game_id)

    def _finish_game(self, status: str, winner: str = "") -> None:
        winner_name = str(winner).upper()
        status = str(status or "finished")

        with self._lock:
            if not self._runtime.game_active:
                return

            k9_colour_name = self._runtime.k9_colour
            game_id = self._runtime.game_id
            player_name = self._runtime.player_name

            if winner_name:
                result_text = f"{winner_name}:{status}"
            else:
                result_text = status.upper()

            self._runtime.state = State.FINISHED
            self._runtime.game_active = False
            self._runtime.game_suspended = False
            self._runtime.engine_busy = False
            self._runtime.pending_move = ""
            self._runtime.result = result_text
            self._clear_pending_evaluation_locked()

        self._cancel_engine_goal()

        if winner_name and winner_name == k9_colour_name:
            hint = "The game is over. I have won."
        elif winner_name:
            hint = "Congratulations. You have won."
        else:
            hint = "The game is drawn."

        self._publish_event(
            "GAME_FINISHED",
            game_id=game_id,
            player_name=player_name,
            status=status,
            colour=winner_name,
            speech_hint=hint,
            message=result_text,
        )
        self._publish_status()

        try:
            if self._game_response is not None:
                self._game_response.close()
        except Exception:
            pass

    def _set_start_error(self, message: str) -> None:
        with self._lock:
            self._runtime.state = State.ERROR
            self._runtime.game_active = False
            self._runtime.error = message
        self._publish_event("GAME_START_ERROR", message=message)
        self._publish_status()

    def _set_runtime_error(self, message: str, fatal: bool) -> None:
        with self._lock:
            self._runtime.error = message
            if fatal:
                self._runtime.state = State.ERROR
                self._runtime.game_active = False
        self._publish_event("CHESS_ERROR", message=message)
        self._publish_status()

    # ------------------------------------------------------------------
    # Message construction
    # ------------------------------------------------------------------

    def _status_message(self) -> ChessStatus:
        with self._lock:
            state = RuntimeState(**vars(self._runtime))

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
        msg.evaluation_valid = state.evaluation_valid
        msg.evaluation_pawns = float(state.evaluation_pawns)
        msg.evaluation_is_mate = state.evaluation_is_mate
        msg.mate_in = int(state.mate_in)
        return msg

    def _publish_status(self) -> None:
        self._status_pub.publish(self._status_message())

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
        with self._lock:
            current_game_id = self._runtime.game_id
            current_player = self._runtime.player_name

        msg = ChessEvent()
        msg.stamp = self.get_clock().now().to_msg()
        msg.type = str(event_type)
        msg.game_id = current_game_id if game_id is None else str(game_id)
        msg.player_name = (
            current_player if player_name is None else str(player_name)
        )
        msg.status = str(status)
        msg.colour = str(colour)
        msg.uci = str(uci)
        msg.san = str(san)
        msg.piece = str(piece)
        msg.from_square = str(from_square)
        msg.to_square = str(to_square)
        msg.captured_piece = str(captured_piece)
        msg.gives_check = bool(gives_check)
        msg.gives_mate = bool(gives_mate)
        msg.source = str(source)
        msg.evaluation_before_valid = bool(evaluation_before_valid)
        msg.evaluation_before_pawns = float(evaluation_before_pawns)
        msg.evaluation_after_valid = bool(evaluation_after_valid)
        msg.evaluation_after_pawns = float(evaluation_after_pawns)
        msg.evaluation_delta_pawns = float(evaluation_delta_pawns)
        msg.is_mate = bool(is_mate)
        msg.mate_in = int(mate_in)
        msg.speech_hint = str(speech_hint)
        msg.message = str(message)
        self._event_pub.publish(msg)

    def _publish_move_event(
        self,
        event_type: str,
        facts: MoveFacts,
        **extra,
    ) -> None:
        self._publish_event(
            event_type,
            uci=facts.uci,
            san=facts.san,
            piece=facts.piece,
            from_square=facts.from_square,
            to_square=facts.to_square,
            captured_piece=facts.captured_piece,
            gives_check=facts.gives_check,
            gives_mate=facts.gives_mate,
            **extra,
        )

    # ------------------------------------------------------------------
    # Evaluation/pending helpers
    # ------------------------------------------------------------------

    def _commit_pending_evaluation_locked(self) -> None:
        self._runtime.evaluation_valid = self._pending_result_eval_valid
        self._runtime.evaluation_pawns = self._pending_result_eval
        self._runtime.evaluation_is_mate = self._pending_result_is_mate
        self._runtime.mate_in = self._pending_result_mate_in

        self._last_post_k9_eval_valid = self._pending_result_eval_valid
        self._last_post_k9_eval = self._pending_result_eval
        self._clear_pending_evaluation_locked()

    def _clear_pending_evaluation_locked(self) -> None:
        self._pending_result_eval_valid = False
        self._pending_result_eval = 0.0
        self._pending_result_is_mate = False
        self._pending_result_mate_in = 0

    def _clear_pending_locked(self) -> None:
        self._runtime.pending_move = ""
        self._runtime.engine_busy = False
        self._engine_goal_handle = None
        self._engine_goal_fen = ""
        self._clear_pending_evaluation_locked()

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def destroy_node(self):
        self._shutdown_event.set()
        self._cancel_engine_goal()

        for response in (self._event_response, self._game_response):
            try:
                if response is not None:
                    response.close()
            except Exception:
                pass

        try:
            self._engine_client.destroy()
        except Exception:
            pass

        if self._bot_api is not None:
            self._bot_api.close()

        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ChessManagerNode()
    executor = MultiThreadedExecutor(num_threads=4)
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
