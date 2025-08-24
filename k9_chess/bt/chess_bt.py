# k9_chess/bt/chess_bt.py
import rclpy
from rclpy.node import Node
from k9_chess_interfaces.srv import GetState, SendMove
from k9_chess_interfaces.action import ComputeMove
import py_trees
import asyncio

class ChessBT(Node):
    def __init__(self):
        super().__init__('chess_behavior_tree')
        self._get_state_cli = self.create_client(GetState, 'chess/get_state')
        self._send_move_cli = self.create_client(SendMove, 'chess/send_move')
        self._compute_move_ac = ActionClientWrapper(self, ComputeMove, 'chess/compute_move')
        self.get_logger().info("Chess BT node ready.")

        # Build BT
        self.tree = self.create_tree()

    def create_tree(self):
        root = py_trees.composites.Sequence("ChessRoot")

        wait_game = py_trees.behaviours.CheckBlackboardVariable(
            name="WaitGameStart",
            variable_name="game_started",
            expected_value=True
        )

        take_turn = py_trees.composites.Sequence("TakeTurn")

        check_my_turn = py_trees.behaviours.CheckBlackboardVariable(
            name="CheckMyTurn",
            variable_name="my_turn",
            expected_value=True
        )

        compute_move = ComputeMoveLeaf("ComputeMove", self._compute_move_ac)
        send_move = SendMoveLeaf("SendMove", self._send_move_cli)

        take_turn.add_children([check_my_turn, compute_move, send_move])
        root.add_children([wait_game, take_turn])

        return root

    async def tick_loop(self):
        while rclpy.ok():
            # Update blackboard
            state = await self.get_state()
            bb = py_trees.blackboard.Blackboard()
            bb.set("game_started", state.status == "started")
            bb.set("my_turn", state.my_turn)
            bb.set("fen", state.fen)
            bb.set("game_id", state.game_id)

            self.tree.tick_once()
            await asyncio.sleep(0.5)

    async def get_state(self):
        while not self._get_state_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for GetState service...")
        req = GetState.Request()
        future = self._get_state_cli.call_async(req)
        await asyncio.wrap_future(future)
        return future.result()

class ComputeMoveLeaf(py_trees.behaviour.Behaviour):
    def __init__(self, name, ac):
        super().__init__(name)
        self._ac = ac

    def update(self):
        bb = py_trees.blackboard.Blackboard()
        fen = bb.get("fen")
        goal = ComputeMove.Goal()
        goal.fen = fen
        goal.use_book = True
        goal.think_time_sec = 2.0

        result = self._ac.send_goal(goal)
        if result:
            bb.set("best_move", result.best_move_uci)
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class SendMoveLeaf(py_trees.behaviour.Behaviour):
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