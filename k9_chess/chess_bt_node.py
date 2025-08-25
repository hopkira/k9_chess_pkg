# k9_chess/bt/chess_bt.py
import rclpy
from rclpy.node import Node
from k9_chess_interfaces.srv import GetState, SendMove
from k9_chess_interfaces.action import ComputeMove
import py_trees
import asyncio
from k9_chess.action_client_wrapper import ActionClientWrapper  # Updated import

class ChessBT(Node):
    """
    Behavior Tree node for playing chess on Lichess.

    - Uses a ComputeMoveLeaf to request moves from Stockfish in infinite mode.
    - Uses SendMoveLeaf to publish moves to Lichess via a service.
    - Ticks asynchronously to allow conversation or other behaviors to interrupt.
    """
    def __init__(self):
        super().__init__('chess_behavior_tree')
        self._get_state_cli = self.create_client(GetState, 'chess/get_state')
        self._send_move_cli = self.create_client(SendMove, 'chess/send_move')
        self._compute_move_ac = ActionClientWrapper(self, ComputeMove, 'chess/compute_move')
        self.get_logger().info("Chess BT node ready.")

        # Build BT
        self.tree = self.create_tree()

    def create_tree(self):
        """Builds the py_trees behavior tree for playing chess."""
        root = py_trees.composites.Sequence("ChessRoot")

        # Wait until a game has started
        wait_game = py_trees.behaviours.CheckBlackboardVariable(
            name="WaitGameStart",
            variable_name="game_started",
            expected_value=True
        )

        take_turn = py_trees.composites.Sequence("TakeTurn")

        # Check if it is our turn
        check_my_turn = py_trees.behaviours.CheckBlackboardVariable(
            name="CheckMyTurn",
            variable_name="my_turn",
            expected_value=True
        )

        # Compute move (infinite mode) and send move
        compute_move = ComputeMoveLeaf("ComputeMove", self._compute_move_ac)
        send_move = SendMoveLeaf("SendMove", self._send_move_cli)

        take_turn.add_children([check_my_turn, compute_move, send_move])
        root.add_children([wait_game, take_turn])
        return root

    async def tick_loop(self):
        """Main asynchronous tick loop."""
        while rclpy.ok():
            # Update blackboard with current game state
            state = await self.get_state()
            bb = py_trees.blackboard.Blackboard()
            bb.set("game_started", state.status == "started")
            bb.set("my_turn", state.my_turn)
            bb.set("fen", state.fen)
            bb.set("game_id", state.game_id)

            # Tick the tree once
            self.tree.tick_once()
            await asyncio.sleep(0.5)

    async def get_state(self):
        """Request the latest chess state from the ChessStateNode."""
        while not self._get_state_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for GetState service...")
        req = GetState.Request()
        future = self._get_state_cli.call_async(req)
        await asyncio.wrap_future(future)
        return future.result()


class ComputeMoveLeaf(py_trees.behaviour.Behaviour):
    """
    Leaf node that requests a move from ChessEngineNode in infinite mode.
    Allows the tree to continue ticking while Stockfish computes.
    """
    def __init__(self, name, ac, max_think_time=None):
        super().__init__(name)
        self._ac = ac
        self._max_think_time = max_think_time
        self._goal_handle = None
        self._start_time = None
        self._move_found = False

    def initialise(self):
        """Send goal to engine when leaf first ticks."""
        bb = py_trees.blackboard.Blackboard()
        fen = bb.get("fen")
        if not fen:
            self._move_found = False
            return

        goal = ComputeMove.Goal()
        goal.fen = fen
        goal.use_book = True
        goal.infinite_mode = True  # Stockfish infinite mode

        self._goal_handle = self._ac.send_goal_async(goal)
        self._start_time = self.get_time()
        self._move_found = False

    def update(self):
        """Check if move is ready, still computing, or timed out."""
        if not self._goal_handle:
            return py_trees.common.Status.FAILURE

        if self._goal_handle.done():
            result = self._goal_handle.result()
            if result:
                bb = py_trees.blackboard.Blackboard()
                bb.set("best_move", result.best_move_uci)
                self._move_found = True
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE

        # Optional max thinking time
        if self._max_think_time and (self.get_time() - self._start_time) > self._max_think_time:
            self._ac.cancel_goal(self._goal_handle)
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def get_time(self):
        import time
        return time.time()


class SendMoveLeaf(py_trees.behaviour.Behaviour):
    """
    Leaf node that sends the computed move to Lichess using the SendMove service.
    """
    def __init__(self, name, client):
        super().__init__(name)
        self._client = client

    def update(self):
        bb = py_trees.blackboard.Blackboard()
        move = bb.get("best_move")
        game_id = bb.get("game_id")
        if not move or not game_id:
            return py_trees.common.Status.FAILURE

        req = SendMove.Request()
        req.game_id = game_id
        req.uci_move = move

        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self._client, future)
        if future.result() and future.result().ok:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


def main():
    rclpy.init()
    node = ChessBT()
    try:
        asyncio.run(node.tick_loop())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()