# k9_chess/move_sender_node.py
import os
import rclpy
from rclpy.node import Node
from k9_chess_interfaces.srv import SendMove
from k9_chess.lichess import LichessAPI

class MoveSenderNode(Node):
    def __init__(self):
        super().__init__('chess_move_sender')

        self._bot_token = os.getenv('LICHESS_BOT_TOKEN')
        self._api = LichessAPI(token=self._bot_token, url='https://lichess.org/api/')

        # Service to send move
        self._srv = self.create_service(SendMove, 'chess/send_move', self.send_move_cb)
        self.get_logger().info("MoveSenderNode ready.")

    def send_move_cb(self, request, response):
        game_id = request.game_id
        uci_move = request.uci_move

        if not game_id or not uci_move:
            response.ok = False
            response.message = "Missing game_id or uci_move"
            return response

        try:
            self._api.make_move(game_id=game_id, move=uci_move)
            response.ok = True
            response.message = f"Move {uci_move} sent for game {game_id}"
            self.get_logger().info(response.message)
        except Exception as e:
            response.ok = False
            response.message = f"Failed to send move: {e}"
            self.get_logger().error(response.message)

        return response

def main():
    rclpy.init()
    node = MoveSenderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()