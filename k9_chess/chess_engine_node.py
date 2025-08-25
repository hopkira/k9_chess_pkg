# k9_chess/chess_engine_node.py
import pathlib
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from k9_chess_interfaces.action import ComputeMove
import chess
import chess.engine
import chess.polyglot

class ChessEngineNode(Node):
    """
    ROS2 node providing chess engine capabilities.

    Uses the Titan binary as the primary engine and Stockfish for analysis.
    Accepts ComputeMove goals and returns best moves with evaluation info.
    """

    def __init__(self):
        super().__init__('chess_engine')

        # Assets directory
        assets_dir = pathlib.Path(get_package_share_directory('k9_chess')) / 'assets'
        titan_path = assets_dir / 'Titans.bin'
        stockfish_path = assets_dir / 'stockfish'

        # Validate files
        if not titan_path.exists():
            raise FileNotFoundError(f"Chess engine not found in assets: {titan_path}")
        if not stockfish_path.exists():
            self.get_logger().warn(f"Stockfish not found in assets: {stockfish_path}")
        if not self._book_path.exists():
            self.get_logger().warn(f"No polyglot book found at {self._book_path}")

        # Launch Titan engine
        self.get_logger().info(f"Starting Titan engine from {titan_path}")
        self._engine = chess.engine.SimpleEngine.popen_uci(str(titan_path))

        # Action server
        self._action = ActionServer(
            self,
            ComputeMove,
            'chess/compute_move',
            execute_callback=self.execute_cb,
            cancel_callback=self.cancel_cb
        )

        self.get_logger().info("ChessEngineNode ready.")

    def cancel_cb(self, goal_handle):
        """Handle action cancellations."""
        return CancelResponse.ACCEPT

    async def execute_cb(self, goal_handle):
        """Compute a move asynchronously for the given FEN."""
        goal = goal_handle.request
        board = chess.Board(goal.fen)
        feedback = ComputeMove.Feedback()
        result = ComputeMove.Result()

        best_move = None
        eval_cp = 0.0
        is_mate = False
        mate_in = 0

        # 1. Try opening book
        if goal.use_book and self._book_path.exists():
            try:
                with chess.polyglot.open_reader(self._book_path) as reader:
                    entry = reader.weighted_choice(board)
                    best_move = entry.move
                    self.get_logger().info(f"Book move chosen: {best_move.uci()}")
            except Exception as e:
                self.get_logger().warn(f"No book move available: {e}")

        # 2. Titan engine analysis if no book move
        if not best_move:
            try:
                limit = chess.engine.Limit(time=goal.think_time_sec or 1.0)
                info = await self._engine.analyse(board, limit, info=chess.engine.INFO_ALL)
                move = info.get("pv", [None])[0]
                score = info["score"].pov(board.turn)
                best_move = move

                if score.is_mate():
                    is_mate = True
                    mate_in = score.mate()
                    eval_cp = 0.0
                else:
                    eval_cp = score.score(mate_score=100000) / 100.0

                # Publish feedback
                feedback.depth = info.get("depth", 0)
                feedback.nps = info.get("nps", 0.0)
                feedback.eval_cp_live = eval_cp
                goal_handle.publish_feedback(feedback)

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
        """Properly quit engine before destroying node."""
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