# k9_chess/chess_bt_node.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from k9_chess_interfaces.action import ComputeMove
from k9_chess_interfaces.msg import BoardState
from k9_chess_interfaces.srv import GetState
import py_trees
import asyncio
import threading
from concurrent.futures import ThreadPoolExecutor

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

        # Create background asyncio loop in a thread
        self._loop = asyncio.new_event_loop()
        t = threading.Thread(target=self._loop.run_forever, daemon=True)
        t.start()

    def create_tree(self):
        root = py_trees.composites.Sequence(name="ChessRoot", memory=False)
        selector = py_trees.composites.Selector(name="SelectMove", memory=True)

        compute_move = ChessComputeTitanMove(self)
        update_state = ChessUpdateBoardState(self)

        selector.add_children([compute_move])
        root.add_children([selector, update_state])
        return root

    def tick_tree(self):
        self.tree.tick_once()

    async def get_current_fen(self):
        """Query ChessStateNode for the current FEN."""
        req = GetState.Request()
        future = self._state_client.call_async(req)
        response = await future
        return response.fen if response else None

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

    def publish_move(self, move_uci):
        """Publish the chosen move to ChessStateNode."""
        msg = BoardState()
        msg.fen = move_uci  # BT only publishes move; ChessStateNode updates FEN
        msg.white_to_move = True  # refine based on actual turn
        self._board_pub.publish(msg)
        self.blackboard.set("last_move", move_uci)

# --- Leaf BT nodes ---
class ChessComputeTitanMove(py_trees.behaviour.Behaviour):
    def __init__(self, node):
        super().__init__("ComputeTitanMove")
        self.node = node
        self._running = False

    def update(self):
        if self._running:
            return py_trees.common.Status.RUNNING

        # Schedule async task on node's loop
        self._running = True
        asyncio.run_coroutine_threadsafe(self._do_request(), self.node._loop)
        return py_trees.common.Status.RUNNING

    async def _do_request(self):
        fen = await self.node.get_current_fen()
        if not fen:
            self.node.get_logger().warn("No FEN available from ChessStateNode")
            self._running = False
            return

        result = await self.node.request_move_async(fen, think_time_sec=2.0)
        if result and result.best_move_uci:
            self.node.publish_move(result.best_move_uci)

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