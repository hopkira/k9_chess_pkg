# k9_chess/game_manager_node.py
import os
import json
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from k9_chess_interfaces.msg import GameEvent, BoardState
from k9_chess_interfaces.srv import StartGame, StopGame
from k9_chess.lichess import LichessAPI
import chess


class GameManagerNode(Node):
    """
    ROS 2 node that manages interaction between the robot and Lichess.

    Responsibilities:
    - Connect to the Lichess API and listen for incoming events (challenges, game starts, etc.).
    - Accept challenges automatically.
    - Start a stream of game updates when a game begins.
    - Maintain and publish the current board state (as FEN) and game events.
    - Provide services to start or stop games (challenge players, resign, abort).

    Design notes:
    - Lichess provides a long-lived HTTP stream for lobby events and per-game updates.
    - These are read incrementally using `.iter_lines()`, which is a blocking generator.
    - To keep the ROS 2 executor responsive, the code uses ROS timers that periodically
      "drain" a few lines from each stream. This avoids blocking the main thread.
    """

    def __init__(self):
        """Initialize publishers, services, API, and stream timers."""
        super().__init__('chess_game_manager')

        # API credentials are expected in environment variables
        self._bot_token = os.getenv('LICHESS_BOT_TOKEN')
        self._player_token = os.getenv('LICHESS_PLAYER_TOKEN')
        self._username = os.getenv('LICHESS_USERNAME')
        self._api = LichessAPI(token=self._bot_token, url='https://lichess.org/api/')

        # Publishers
        self._event_pub = self.create_publisher(GameEvent, 'chess/game_event', 10)
        self._board_pub = self.create_publisher(BoardState, 'chess/board_state', 10)

        # Services
        self._srv_start = self.create_service(StartGame, 'chess/start_game', self.start_game_cb)
        self._srv_stop = self.create_service(StopGame, 'chess/stop_game', self.stop_game_cb)

        # Game state
        self._game_id = None
        self._assigned_color = None
        self._board = chess.Board()

        # Event stream objects
        self._stream = None
        self._lines = None

        # Timer: poll the Lichess lobby event stream at 10 Hz
        # This timer ensures we remain responsive by reading only a few lines per tick
        self._timer = self.create_timer(0.1, self.ensure_stream)

    def ensure_stream(self):
        """
        Ensure the Lichess lobby event stream is connected and read incrementally.

        Called repeatedly by a ROS timer:
        - If no stream is active, attempt to connect.
        - If connected, read a few lines from the stream and handle any events.
        - Keeps the main thread responsive by not blocking indefinitely.
        """
        if self._stream is None:
            try:
                self._stream = self._api.get_event_stream()
                self._lines = self._stream.iter_lines()
            except Exception as e:
                self.get_logger().warn(f"Retrying Lichess stream: {e}")
                return

        # Drain a few lines per tick to avoid blocking
        for _ in range(5):
            try:
                line = next(self._lines)
            except StopIteration:
                # Stream ended; reset so we reconnect next tick
                self._stream = None
                self._lines = None
                return
            except Exception:
                return

            if not line:
                return

            try:
                evt = json.loads(line.decode('utf-8'))
            except Exception as e:
                self.get_logger().warn(f"Malformed Lichess event: {e}")
                continue

            self.handle_event(evt)

    def handle_event(self, evt):
        """
        Handle lobby-level events from the Lichess event stream.
        Enforces one active game at a time:
          - If already in a game, decline new challenges.
          - Otherwise, accept the first challenge and track it.
        """
        msg = GameEvent()
        msg.type = evt.get('type', '')

        if msg.type == 'challenge':
            ch_id = evt['challenge']['id']

            # Already in a game? Decline politely
            if self._game_id:
                try:
                    self._api.decline_challenge(ch_id)
                    self.get_logger().info(f"Declined challenge {ch_id} (already in a game).")
                except Exception as e:
                    self.get_logger().warn(f"Failed to decline challenge {ch_id}: {e}")
                return

            # Otherwise, accept the first challenge
            try:
                self._api.accept_challenge(ch_id)
                self.get_logger().info(f"Accepted challenge {ch_id}")
            except Exception as e:
                self.get_logger().error(f"Failed to accept challenge {ch_id}: {e}")

        elif msg.type == 'gameStart':
            # Only track the first game — ignore extra starts if somehow triggered
            if self._game_id:
                self.get_logger().warn("Received gameStart while already in a game, ignoring.")
                return

            self._game_id = evt['game']['id']
            self._assigned_color = evt['game'].get('color')
            msg.game_id = self._game_id
            msg.color = self._assigned_color
            self._event_pub.publish(msg)

            # Start per-game stream in another timer
            self.create_timer(0.05, self.stream_game_once)
            self.get_logger().info(f"Game {self._game_id} started as {self._assigned_color}")

        else:
            # Forward other lobby-level events if desired
            self._event_pub.publish(msg)

    def stream_game_once(self):
        """
        Read a few lines from the active game stream and publish updates.

        Runs on its own ROS timer, so game updates do not block lobby events.
        """
        if not self._game_id:
            return

        if not hasattr(self, '_game_stream'):
            try:
                self._game_stream = self._api.get_stream(self._game_id)
                self._game_lines = self._game_stream.iter_lines()
                # The first line usually contains initial game state
                first = json.loads(next(self._game_lines).decode('utf-8'))
                state = first.get('state', {})
                self.publish_state(state)
            except Exception as e:
                self.get_logger().error(f"game stream error: {e}")
                return

        # Pull several updates per tick for responsiveness
        for _ in range(10):
            try:
                chunk = next(self._game_lines)
            except StopIteration:
                return
            if not chunk:
                return

            try:
                obj = json.loads(chunk.decode('utf-8'))
            except Exception:
                continue

            if obj.get('type') == 'gameState':
                self.publish_state(obj)

    def publish_state(self, state):
        """
        Publish the current board state and a `gameState` event message.

        Args:
            state (dict): Lichess game state JSON, contains move list and status.
        """
        moves_str = state.get('moves', '')
        status = state.get('status', '')

        # Reconstruct board from move history
        board = chess.Board()
        if moves_str:
            for m in moves_str.split():
                try:
                    board.push(chess.Move.from_uci(m))
                except Exception:
                    self.get_logger().warn(f"Illegal move ignored: {m}")

        # Publish BoardState
        bmsg = BoardState()
        bmsg.fen = board.fen()
        bmsg.game_id = self._game_id or ''
        bmsg.white_to_move = board.turn
        self._board_pub.publish(bmsg)

        # Publish GameEvent of type 'gameState'
        emsg = GameEvent()
        emsg.type = 'gameState'
        emsg.game_id = self._game_id or ''
        emsg.status = status
        self._event_pub.publish(emsg)

        # NOTE: Opponent moves could be detected by comparing move counts,
        # but this is left as optional.

    def start_game_cb(self, req, resp):
        """
        Service callback: create and send a Lichess challenge.

        Args:
            req (StartGame.Request): Request with rated flag, clock settings, and color.
            resp (StartGame.Response): Response with game_id and status message.
        """
        try:
            params = {
                'rated': req.rated,
                'variant': 'standard',
                'clock.limit': str(req.clock_limit or 300),
                'clock.increment': str(req.clock_increment or 5),
                'color': req.color or 'random',
            }
            r = self._api.create_challenge(self._username, params)
            gid = (r.get('game') or {}).get('id', '')
            resp.ok = True
            resp.game_id = gid
            resp.message = 'challenge sent'
        except Exception as e:
            resp.ok = False
            resp.message = str(e)
        return resp

    def stop_game_cb(self, req, resp):
        """
        Service callback: stop a game (resign or abort).

        Args:
            req (StopGame.Request): Contains game_id and reason ('resign' or 'abort').
            resp (StopGame.Response): Response indicating success/failure.
        """
        try:
            if req.reason == 'resign':
                self._api.resign(req.game_id)
            elif req.reason == 'abort':
                self._api.abort(req.game_id)
            resp.ok = True
            resp.message = 'ok'
        except Exception as e:
            resp.ok = False
            resp.message = str(e)
        return resp


def main():
    """Entry point for the Chess Game Manager node."""
    rclpy.init()
    node = GameManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()