# k9_chess/chess_state_node.py
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from k9_chess_interfaces.msg import BoardState, GameEvent
from k9_chess_interfaces.srv import GetState

class ChessStateNode(Node):
    """
    A ROS 2 node that maintains the current chess board state and game status.

    Subscribes to:
        - 'chess/board_state' (BoardState): updates internal board FEN and turn information
        - 'chess/game_event' (GameEvent): updates game status (start, ongoing, end)

    Provides service:
        - 'chess/get_state' (GetState): returns current FEN, game status, whose turn it is, and game ID

    This node decouples game state management from the game logic and allows other nodes
    (like behavior trees or AI nodes) to query the current game state at any time.

    Thread-safety:
        Uses a threading.Lock to ensure that concurrent updates via subscriptions and
        queries via the service do not result in inconsistent state.
    """

    def __init__(self):
        """Initialize the ChessStateNode, its subscriptions/service, and internal state."""
        super().__init__('chess_state')

        # Thread lock to protect shared state
        self._lock = threading.Lock()

        # Internal state
        self._fen = "startpos"      # Standard starting position FEN
        self._status = "idle"       # Game status: idle, started, in-progress, finished
        self._my_turn = False       # True if it's white's turn to move
        self._game_id = ""          # Current active game ID

        # Subscriptions
        self.create_subscription(BoardState, 'chess/board_state', self.board_cb, 10)
        self.create_subscription(GameEvent, 'chess/game_event', self.event_cb, 10)

        # Service to query current state
        self._srv = self.create_service(GetState, 'chess/get_state', self.get_state_cb)

    def board_cb(self, msg: BoardState):
        """
        Callback for updates to the board state.

        Args:
            msg (BoardState): contains FEN, game ID, and whose turn it is.
        """
        with self._lock:
            old_fen = self._fen
            old_turn = self._my_turn
            self._fen = msg.fen
            self._game_id = msg.game_id
            self._my_turn = msg.white_to_move  # True if it's white's turn to move

            self.get_logger().debug(
                f"[BoardState update] FEN: {old_fen} -> {self._fen}, "
                f"My turn: {old_turn} -> {self._my_turn}, GameID: {self._game_id}"
            )

    def event_cb(self, msg: GameEvent):
        """
        Callback for game events to update internal status.

        Args:
            msg (GameEvent): contains type of event and optional game ID/status.
        """
        with self._lock:
            old_status = self._status
            old_game_id = self._game_id

            if msg.type == "gameState":
                self._status = msg.status
            elif msg.type == "gameStart":
                self._status = "started"
                self._game_id = msg.game_id
            elif msg.type == "gameEnd":
                self._status = msg.status
            elif msg.type == "challenge":
                # Challenge received but not yet accepted; keep status as idle
                pass

            self.get_logger().debug(
                f"[GameEvent update] Type: {msg.type}, Status: {old_status} -> {self._status}, "
                f"GameID: {old_game_id} -> {self._game_id}"
            )

    def get_state_cb(self, request, response):
        """
        Service callback to provide current game state.

        Args:
            request: GetState request (not used; always returns current state)
            response: GetState response to populate with current FEN, status, turn, and game ID

        Returns:
            GetState.Response: populated with current game state
        """
        with self._lock:
            response.fen = self._fen
            response.status = self._status
            response.my_turn = self._my_turn
            response.game_id = self._game_id

            self.get_logger().debug(
                f"[GetState] FEN: {response.fen}, Status: {response.status}, "
                f"My turn: {response.my_turn}, GameID: {response.game_id}"
            )

        return response

def main():
    """
    Entry point for the ChessStateNode.

    Uses a MultiThreadedExecutor to allow subscriptions and service callbacks to
    operate in parallel without blocking each other.
    """
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