import os
import pathlib
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from k9_chess_interfaces.action import ComputeMove
import chess
import chess.engine
import chess.polyglot

class ChessEngineNode(Node):
    """
    ROS 2 node that wraps the Stockfish/Titan chess engine and exposes
    a ComputeMove action server.
    The engine binary is always loaded from the installed 'assets' folder.
    """
    def __init__(self):
        super().__init__('chess_engine')

        # Locate the engine binary in the installed assets directory
        pkg_dir = pathlib.Path(__file__).parent.parent  # k9_chess/
        engine_path = pkg_dir / 'assets' / 'Titans.bin'

        # Verify that the binary exists
        if not engine_path.exists():
            raise FileNotFoundError(f"Chess engine not found in assets: {engine_path}")

        # Launch Stockfish/Titan via python-chess
        self._engine = chess.engine.SimpleEngine.popen_uci(str(engine_path))

        # Setup ComputeMove action server
        self._action = ActionServer(
            self,
            ComputeMove,
            'chess/compute_move',
            execute_callback=self.execute_cb,
            cancel_callback=self.cancel_cb
        )

        # Optional polyglot opening book (also in assets if needed)
        self._book_path = pkg_dir / 'assets' / 'Titans.bin'
        if not self._book_path.exists():
            self._book_path = None  # No book available

        self.get_logger().info(f"ChessEngineNode ready. Using engine: {engine_path}")

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