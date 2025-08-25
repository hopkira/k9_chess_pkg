import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from k9_chess_interfaces.action import ComputeMove
import chess
import chess.engine
import chess.polyglot
import asyncio

class ChessEngineNode(Node):
    """
    ROS2 node wrapping a chess engine (Stockfish/Titan) providing an action server
    to compute moves. Supports polyglot book moves, infinite analysis with time budget,
    CPU core limiting, and incremental feedback.
    """
    def __init__(self):
        super().__init__('chess_engine')

        # Declare parameter for number of threads (CPU cores) to use
        self.declare_parameter('num_threads', 2)
        self.num_threads = self.get_parameter('num_threads').value

        # Paths from assets folder (installed via package.xml)
        stockfish_path = os.path.join(
            os.path.dirname(__file__), '..', 'assets', 'stockfish'
        )
        titan_path = os.path.join(
            os.path.dirname(__file__), '..', 'assets', 'titan'
        )

        # Default to Stockfish
        engine_path = stockfish_path if os.path.exists(stockfish_path) else titan_path
        self.get_logger().info(f"Starting chess engine at: {engine_path} using {self.num_threads} threads")

        # Start engine with thread constraint
        self._engine = chess.engine.SimpleEngine.popen_uci(
            engine_path, setpgrp=True  # separate process group for better signal handling
        )
        self._engine.configure({'Threads': self.num_threads})

        # Path to polyglot book
        self._book_path = os.path.join(
            os.path.dirname(__file__), '..', 'assets', 'book.bin'
        )

        # Action server for ComputeMove
        self._action = ActionServer(
            self,
            ComputeMove,
            'chess/compute_move',
            execute_callback=self.execute_cb,
            cancel_callback=self.cancel_cb
        )

        self.get_logger().info("ChessEngineNode ready.")

    def cancel_cb(self, goal_handle):
        """
        Accept all cancel requests immediately.
        """
        return CancelResponse.ACCEPT

    async def execute_cb(self, goal_handle):
        """
        Execute a move computation request. Supports:
        - Polyglot book moves
        - Infinite analysis constrained by think_time_sec
        - CPU thread limiting
        - Incremental feedback for BT integration
        """
        goal = goal_handle.request
        board = chess.Board(goal.fen)
        feedback = ComputeMove.Feedback()
        result = ComputeMove.Result()

        best_move = None
        eval_cp = 0.0
        is_mate = False
        mate_in = 0

        # 1. Try opening book move
        if goal.use_book and os.path.exists(self._book_path):
            try:
                with chess.polyglot.open_reader(self._book_path) as reader:
                    entry = reader.weighted_choice(board)
                    best_move = entry.move
                    self.get_logger().info(f"Book move chosen: {best_move.uci()}")
            except Exception as e:
                self.get_logger().warn(f"No book move available: {e}")

        # 2. Infinite analysis with time budget
        if not best_move:
            think_time = goal.think_time_sec or 1.0
            start_time = self.get_clock().now().nanoseconds / 1e9

            try:
                analysis = self._engine.analysis(
                    board,
                    chess.engine.Limit(infinite=True)
                )

                async for info in analysis:
                    # Update best move from principal variation
                    if "pv" in info and info["pv"]:
                        move = info["pv"][0]
                        score = info.get("score", None)
                        if score:
                            pov_score = score.pov(board.turn)
                            if pov_score.is_mate():
                                is_mate = True
                                mate_in = pov_score.mate()
                                eval_cp = 0.0
                            else:
                                eval_cp = pov_score.score(mate_score=100000) / 100.0

                        feedback.eval_cp_live = eval_cp
                        goal_handle.publish_feedback(feedback)
                        best_move = move

                    # Stop if time budget exceeded
                    elapsed = self.get_clock().now().nanoseconds / 1e9 - start_time
                    if elapsed >= think_time:
                        break

                # Stop the engine search
                await analysis.stop()

            except Exception as e:
                self.get_logger().error(f"Engine analysis failed: {e}")
                goal_handle.abort()
                return ComputeMove.Result()

        # 3. Populate result
        result.best_move_uci = best_move.uci() if best_move else ""
        result.eval_cp = eval_cp
        result.is_mate = is_mate
        result.mate_in = mate_in

        goal_handle.succeed()
        return result

    def destroy_node(self):
        """
        Cleanly terminate engine subprocess on node destruction.
        """
        try:
            self._engine.quit()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ChessEngineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()