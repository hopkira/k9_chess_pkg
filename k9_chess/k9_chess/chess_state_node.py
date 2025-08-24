# k9_chess/chess_state_node.py
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from k9_chess_interfaces.msg import BoardState, GameEvent
from k9_chess_interfaces.srv import GetState

class ChessStateNode(Node):
    def __init__(self):
        super().__init__('chess_state')

        # Internal state
        self._fen = "startpos"
        self._status = "idle"
        self._my_turn = False
        self._game_id = ""

        # Subscriptions
        self.create_subscription(BoardState, 'chess/board_state', self.board_cb, 10)
        self.create_subscription(GameEvent, 'chess/game_event', self.event_cb, 10)

        # Service
        self._srv = self.create_service(GetState, 'chess/get_state', self.get_state_cb)

    def board_cb(self, msg: BoardState):
        self._fen = msg.fen
        self._game_id = msg.game_id
        self._my_turn = msg.white_to_move  # true if white to move
        # status updated from GameEvent
        self.get_logger().debug(f"Board updated: {self._fen}")

    def event_cb(self, msg: GameEvent):
        if msg.type == "gameState":
            self._status = msg.status
        elif msg.type == "gameStart":
            self._status = "started"
            self._game_id = msg.game_id
        elif msg.type == "gameEnd":
            self._status = msg.status
        elif msg.type == "challenge":
            # keep as idle until accepted
            pass

    def get_state_cb(self, request, response):
        response.fen = self._fen
        response.status = self._status
        response.my_turn = self._my_turn
        response.game_id = self._game_id
        return response

def main():
    rclpy.init()
    node = ChessStateNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()