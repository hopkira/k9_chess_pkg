# k9_chess/chess_bt_node.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from k9_chess_interfaces.action import ComputeMove
from k9_chess_interfaces.msg import BoardState
from k9_chess_interfaces.srv import GetState
import py_trees
from rclpy.task import create_task
import chess

class ChessBT(Node):
    def __init__(self):
        super().__init__('chess_bt')

        # Async Action client for ComputeMove
        self._action_client = ActionClient(self, ComputeMove, 'chess/compute_move')

        # Publisher to update board state
        self._board_pub = self.create_publisher(BoardState, 'chess/board_state', 10)

        # Client to query current state
        self._state_client = self.create_client(GetState, 'chess/get_state')
        while not self._state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for ChessState service...")

        # Blackboard for BT
        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.set("last_move", None)

        # Build the behavior tree
        self.tree = self.create_tree()

        # Timer to tick the BT
        self._timer = self.create_timer(0.5, self.tick_tree)

    def create_tree(self):
        root = py_trees.composites.Sequence(name="ChessRoot", memory=False)
        selector = py_trees.composites.Selector(name="SelectMove", memory=True)

        compute_move = ChessComputeMove(self)
        update_state = ChessUpdateBoardState(self)

        selector.add_children([compute_move])
        root.add_children([selector, update_state])
        return root

    def tick_tree(self):
        self.tree.tick_once()

    async def get_current_state(self):
        """Query ChessStateNode for current FEN and turn."""
        req = GetState.Request()
        future = self._state_client.call_async(req)
        response = await future
        if response:
            return response.fen, response.my_turn
        return None, None

    async def request_move_async(self, fen, think_time_sec=5.0):
        """Request a move from the chess engine."""
        goal_msg = ComputeMove.Goal()
        goal_msg.fen = fen
        goal_msg.think_time_sec = think_time_sec

        future = self._action_client.send_goal_async(goal_msg)
        goal_handle = await future
        if not goal_handle.accepted:
            self.get_logger().error("ComputeMove goal rejected")
            return None

        result_future = goal_handle.get_result_async()
        result = await result_future
        return result.result

    def publish_move(self, move_uci, current_fen, white_to_move):
        """Apply move to FEN, compute new FEN, and publish to ChessStateNode."""
        board = chess.Board(current_fen)
        try:
            move = chess.Move.from_uci(move_uci)
            if move not in board.legal_moves:
                self.get_logger().warn(f"Illegal move attempted: {move_uci}")
                return
            board.push(move)
        except Exception as e:
            self.get_logger().error(f"Failed to apply move {move_uci}: {e}")
            return

        msg = BoardState()
        msg.fen = board.fen()
        msg.white_to_move = board.turn  # Automatically correct turn
        self._board_pub.publish(msg)
        self.blackboard.set("last_move", move_uci)
        self.get_logger().info(f"Published move {move_uci}, new FEN: {msg.fen}")

# --- Leaf BT nodes ---
class ChessComputeMove(py_trees.behaviour.Behaviour):
    def __init__(self, node):
        super().__init__("ComputeMove")
        self.node = node
        self._running = False

    def update(self):
        if self._running:
            return py_trees.common.Status.RUNNING

        # Schedule async task to query state and request move
        self._running = True
        create_task(self._do_request())
        return py_trees.common.Status.RUNNING

    async def _do_request(self):
        fen, white_to_move = await self.node.get_current_state()
        if not fen:
            self.node.get_logger().warn("No FEN available from ChessStateNode")
            self._running = False
            return

        result = await self.node.request_move_async(fen, think_time_sec=2.0)
        if result and result.best_move_uci:
            self.node.publish_move(result.best_move_uci, fen, white_to_move)

        self._running = False

class ChessUpdateBoardState(py_trees.behaviour.Behaviour):
    def __init__(self, node):
        super().__init__("UpdateBoardState")
        self.node = node

    def update(self):
        last_move = self.node.blackboard.get("last_move")
        if last_move:
            self.node.get_logger().info(f"Last move applied: {last_move}")
        return py_trees.common.Status.SUCCESS

def main(args=None):
    rclpy.init(args=args)
    node = ChessBT()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()