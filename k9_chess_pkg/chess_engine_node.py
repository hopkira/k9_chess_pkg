#!/usr/bin/env python3
"""Cancellable Stockfish/Polyglot ROS 2 action server for K9."""

from __future__ import annotations

import os
from pathlib import Path
import threading

import chess
import chess.engine
import chess.polyglot

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from k9_interfaces_pkg.action import ComputeChessMove


class ChessEngineNode(Node):
    """Provide K9 move selection without owning game/session state."""

    def __init__(self) -> None:
        super().__init__("chess_engine")

        historical_stockfish = "/home/pi/Stockfish-sf_15/src/stockfish"
        historical_book = "/home/pi/k9-chess-angular/Titans.bin"

        self.declare_parameter(
            "stockfish_path",
            os.getenv("STOCKFISH_PATH", historical_stockfish),
        )
        self.declare_parameter(
            "opening_book_path",
            os.getenv("K9_CHESS_BOOK_PATH", historical_book),
        )
        self.declare_parameter("engine_threads", 2)
        self.declare_parameter("engine_hash_mb", 128)
        self.declare_parameter("book_eval_time_sec", 0.15)
        self.declare_parameter("result_eval_time_sec", 0.15)
        self.declare_parameter("warmup_enabled", True)

        self.stockfish_path = Path(
            str(self.get_parameter("stockfish_path").value)
        ).expanduser()
        self.opening_book_path = Path(
            str(self.get_parameter("opening_book_path").value)
        ).expanduser()
        self.engine_threads = max(
            1,
            int(self.get_parameter("engine_threads").value),
        )
        self.engine_hash_mb = max(
            16,
            int(self.get_parameter("engine_hash_mb").value),
        )
        self.book_eval_time_sec = max(
            0.02,
            float(self.get_parameter("book_eval_time_sec").value),
        )
        self.result_eval_time_sec = max(
            0.02,
            float(self.get_parameter("result_eval_time_sec").value),
        )
        self.warmup_enabled = bool(
            self.get_parameter("warmup_enabled").value
        )

        if not self.stockfish_path.is_file():
            raise FileNotFoundError(
                f"Stockfish executable not found: {self.stockfish_path}"
            )

        self.get_logger().info(
            f"Starting Stockfish: {self.stockfish_path}"
        )
        self._engine = chess.engine.SimpleEngine.popen_uci(
            str(self.stockfish_path)
        )
        self._engine_lock = threading.Lock()

        options = {}
        if "Threads" in self._engine.options:
            options["Threads"] = self.engine_threads
        if "Hash" in self._engine.options:
            options["Hash"] = self.engine_hash_mb
        if options:
            self._engine.configure(options)

        if self.opening_book_path.is_file():
            self.get_logger().info(
                f"Polyglot opening book: {self.opening_book_path}"
            )
        else:
            self.get_logger().warning(
                "Opening book not found; Stockfish-only play will be used: "
                f"{self.opening_book_path}"
            )

        if self.warmup_enabled:
            try:
                self._engine.play(
                    chess.Board(),
                    chess.engine.Limit(depth=1),
                )
                self.get_logger().info("Stockfish warm-up complete")
            except Exception as exc:
                self.get_logger().warning(
                    f"Stockfish warm-up failed: {exc}"
                )

        self._callback_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            ComputeChessMove,
            "/chess/compute_move",
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )

        self.get_logger().info("Chess engine action server ready")

    def _goal_callback(self, request) -> GoalResponse:
        try:
            chess.Board(request.fen)
        except ValueError:
            self.get_logger().warning("Rejecting invalid FEN")
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_callback(_goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    @staticmethod
    def _empty_result(error: str = ""):
        result = ComputeChessMove.Result()
        result.success = False
        result.best_move_uci = ""
        result.best_move_san = ""
        result.source = ""
        result.position_evaluation_valid = False
        result.position_eval_pawns = 0.0
        result.position_is_mate = False
        result.position_mate_in = 0
        result.resulting_evaluation_valid = False
        result.resulting_eval_pawns = 0.0
        result.resulting_is_mate = False
        result.resulting_mate_in = 0
        result.error = error
        return result

    def _execute(self, goal_handle):
        request = goal_handle.request
        board = chess.Board(request.fen)
        k9_colour = chess.WHITE if request.k9_is_white else chess.BLACK
        think_time = max(0.05, float(request.think_time_sec or 1.0))

        while not self._engine_lock.acquire(timeout=0.05):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._empty_result("cancelled")

        try:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._empty_result("cancelled")

            best_move = None
            source = ""
            position_valid = False
            position_eval = 0.0
            position_is_mate = False
            position_mate_in = 0

            if request.use_book and self.opening_book_path.is_file():
                try:
                    with chess.polyglot.open_reader(
                        str(self.opening_book_path)
                    ) as reader:
                        entry = reader.weighted_choice(board)
                        best_move = entry.move
                        source = "BOOK"

                    (
                        position_valid,
                        position_eval,
                        position_is_mate,
                        position_mate_in,
                    ) = self._short_evaluate(
                        board,
                        k9_colour,
                        self.book_eval_time_sec,
                    )
                    self.get_logger().info(
                        f"Weighted book move: {best_move.uci()}"
                    )
                except IndexError:
                    best_move = None
                except Exception as exc:
                    self.get_logger().warning(
                        f"Opening book unavailable for position: {exc}"
                    )
                    best_move = None

            if best_move is None:
                (
                    best_move,
                    position_valid,
                    position_eval,
                    position_is_mate,
                    position_mate_in,
                    cancelled,
                ) = self._analyse_for_move(
                    board,
                    k9_colour,
                    think_time,
                    goal_handle,
                )
                source = "STOCKFISH"

                if cancelled:
                    goal_handle.canceled()
                    return self._empty_result("cancelled")

            if best_move is None:
                goal_handle.abort()
                return self._empty_result(
                    "Stockfish returned no legal move"
                )

            if best_move not in board.legal_moves:
                goal_handle.abort()
                return self._empty_result(
                    f"Engine selected illegal move {best_move.uci()}"
                )

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._empty_result("cancelled")

            san = board.san(best_move)
            resulting_board = board.copy(stack=True)
            resulting_board.push(best_move)

            (
                resulting_valid,
                resulting_eval,
                resulting_is_mate,
                resulting_mate_in,
            ) = self._short_evaluate(
                resulting_board,
                k9_colour,
                self.result_eval_time_sec,
            )

            result = ComputeChessMove.Result()
            result.success = True
            result.best_move_uci = best_move.uci()
            result.best_move_san = san
            result.source = source
            result.position_evaluation_valid = position_valid
            result.position_eval_pawns = float(position_eval)
            result.position_is_mate = bool(position_is_mate)
            result.position_mate_in = int(position_mate_in)
            result.resulting_evaluation_valid = resulting_valid
            result.resulting_eval_pawns = float(resulting_eval)
            result.resulting_is_mate = resulting_is_mate
            result.resulting_mate_in = int(resulting_mate_in)
            result.error = ""

            goal_handle.succeed()
            return result

        except Exception as exc:
            self.get_logger().error(f"Chess engine failure: {exc}")
            goal_handle.abort()
            return self._empty_result(str(exc))
        finally:
            self._engine_lock.release()

    def _analyse_for_move(
        self,
        board: chess.Board,
        k9_colour: chess.Color,
        think_time: float,
        goal_handle,
    ) -> tuple[chess.Move | None, bool, float, bool, int, bool]:
        """Run interruptible analysis and return move plus evaluation metadata."""
        latest_info = {}
        best_move = None

        with self._engine.analysis(
            board,
            chess.engine.Limit(time=think_time),
            info=chess.engine.INFO_ALL,
        ) as analysis:
            for info in analysis:
                latest_info = info

                pv = info.get("pv")
                if pv:
                    best_move = pv[0]

                score = info.get("score")
                feedback = ComputeChessMove.Feedback()
                feedback.depth = int(info.get("depth") or 0)
                feedback.nodes_per_second = int(info.get("nps") or 0)
                feedback.evaluation_valid = False
                feedback.eval_pawns_live = 0.0

                if score is not None:
                    pov = score.pov(k9_colour)
                    if not pov.is_mate():
                        cp = pov.score()
                        if cp is not None:
                            feedback.evaluation_valid = True
                            feedback.eval_pawns_live = float(cp) / 100.0

                goal_handle.publish_feedback(feedback)

                if goal_handle.is_cancel_requested:
                    analysis.stop()
                    return None, False, 0.0, False, 0, True

            best = analysis.wait()
            if best_move is None:
                best_move = best.move

        valid = False
        eval_pawns = 0.0
        is_mate = False
        mate_in = 0

        score = latest_info.get("score")
        if score is not None:
            pov = score.pov(k9_colour)
            if pov.is_mate():
                is_mate = True
                mate_in = int(pov.mate() or 0)
            else:
                cp = pov.score()
                if cp is not None:
                    valid = True
                    eval_pawns = float(cp) / 100.0

        return best_move, valid, eval_pawns, is_mate, mate_in, False

    def _short_evaluate(
        self,
        board: chess.Board,
        k9_colour: chess.Color,
        time_sec: float,
    ) -> tuple[bool, float, bool, int]:
        """Return evaluation and mate metadata from K9's point of view."""
        if board.is_game_over(claim_draw=False):
            if board.is_checkmate():
                # Side to move has been mated.
                k9_won = board.turn != k9_colour
                return False, 0.0, True, 0 if k9_won else 0
            return False, 0.0, False, 0

        info = self._engine.analyse(
            board,
            chess.engine.Limit(time=time_sec),
            info=chess.engine.INFO_SCORE,
        )
        score = info.get("score")
        if score is None:
            return False, 0.0, False, 0

        pov = score.pov(k9_colour)
        if pov.is_mate():
            mate_in = pov.mate()
            return False, 0.0, True, int(mate_in or 0)

        cp = pov.score()
        if cp is None:
            return False, 0.0, False, 0

        return True, float(cp) / 100.0, False, 0

    def destroy_node(self):
        try:
            self._action_server.destroy()
        except Exception:
            pass

        try:
            self._engine.quit()
        except Exception:
            pass

        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ChessEngineNode()
    executor = MultiThreadedExecutor(num_threads=3)
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
