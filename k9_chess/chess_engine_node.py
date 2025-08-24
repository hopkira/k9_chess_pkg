# k9_chess/chess_engine_node.py
import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from k9_chess_interfaces.action import ComputeMove
import chess
import chess.engine
import chess.polyglot

class ChessEngineNode(Node):
    def __init__(self):
        super().__init__('chess_engine')
        stockfish_path = os.getenv('STOCKFISH_PATH', 'stockfish')
        self._engine = chess.engine.SimpleEngine.popen_uci(stockfish_path)
        self._action = ActionServer(
            self,
            ComputeMove,
            'chess/compute_move',
            execute_callback=self.execute_cb,
            cancel_callback=self.cancel_cb
        )
        self._book_path = os.getenv('POLYGLOT_BOOK', '')
        self.get_logger().info("ChessEngineNode ready.")

    def cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    async def execute_cb(self, goal_handle):
        goal = goal_handle.request
        board = chess.Board(goal.fen)
        feedback = ComputeMove.Feedback()
        result = ComputeMove.Result()

        best_move = None
        eval_cp = 0.0
        is_mate = False
        mate_in = 0

        # 1. Try book move if requested
        if goal.use_book and self._book_path and os.path.exists(self._book_path):
            try:
                with chess.polyglot.open_reader(self._book_path) as reader:
                    entry = reader.weighted_choice(board)
                    best_move = entry.move
                    self.get_logger().info(f"Book move chosen: {best_move.uci()}")
            except Exception as e:
                self.get_logger().warn(f"No book move: {e}")

        # 2. Otherwise, ask Stockfish
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