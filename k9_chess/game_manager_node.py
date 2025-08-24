# k9_chess/game_manager_node.py
import os
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from k9_chess_interfaces.msg import GameEvent, BoardState
from k9_chess_interfaces.srv import StartGame, StopGame
from k9_chess.lichess import LichessAPI
import chess

class GameManagerNode(Node):
    def __init__(self):
        super().__init__('chess_game_manager')
        self._bot_token = os.getenv('LICHESS_BOT_TOKEN')
        self._player_token = os.getenv('LICHESS_PLAYER_TOKEN')
        self._username = os.getenv('LICHESS_USERNAME')
        self._api = LichessAPI(token=self._bot_token, url='https://lichess.org/api/')

        self._event_pub = self.create_publisher(GameEvent, 'chess/game_event', 10)
        self._board_pub = self.create_publisher(BoardState, 'chess/board_state', 10)

        self._srv_start = self.create_service(StartGame, 'chess/start_game', self.start_game_cb)
        self._srv_stop = self.create_service(StopGame, 'chess/stop_game', self.stop_game_cb)

        self._game_id = None
        self._assigned_color = None
        self._board = chess.Board()

        # Spin event stream in a separate thread using a timer
        self._timer = self.create_timer(0.1, self.ensure_stream)
        self._stream = None
        self._lines = None

    def ensure_stream(self):
        if self._stream is None:
            try:
                self._stream = self._api.get_event_stream()
                self._lines = self._stream.iter_lines()
            except Exception as e:
                self.get_logger().warn(f"Retrying Lichess stream: {e}")
                return
        # Drain a few lines per tick to stay responsive
        for _ in range(5):
            try:
                line = next(self._lines)
            except StopIteration:
                self._stream = None
                self._lines = None
                return
            except Exception:
                return
            if not line:
                return
            line_str = line.decode('utf-8').strip()
            if line_str:
                evt = json.loads(line_str)
            else:
                continue  # skip empty lines
            evt = json.loads(line.decode('utf-8'))
            self.handle_event(evt)

    def handle_event(self, evt):
        msg = GameEvent()
        msg.type = evt.get('type', '')
        if msg.type == 'challenge':
            ch_id = evt['challenge']['id']
            try:
                self._api.accept_challenge(ch_id)
                self.get_logger().info(f"Accepted challenge {ch_id}")
            except Exception as e:
                self.get_logger().error(f"Failed to accept challenge: {e}")
        elif msg.type == 'gameStart':
            self._game_id = evt['game']['id']
            self._assigned_color = evt['game'].get('color')
            msg.game_id = self._game_id
            msg.color = self._assigned_color
            self._event_pub.publish(msg)
            # Start the per-game stream in another timer
            self.create_timer(0.05, self.stream_game_once)
        else:
            # other lobby-level events can be forwarded if desired
            self._event_pub.publish(msg)

    def stream_game_once(self):
        if not self._game_id:
            return
        if not hasattr(self, '_game_stream'):
            try:
                self._game_stream = self._api.get_stream(self._game_id)
                self._game_lines = self._game_stream.iter_lines()
                first = json.loads(next(self._game_lines).decode('utf-8'))
                state = first.get('state', {})
                self.publish_state(state)
            except Exception as e:
                self.get_logger().error(f"game stream error: {e}")
                return
        # Pull several updates per tick
        for _ in range(10):
            try:
                chunk = next(self._game_lines)
            except StopIteration:
                return
            if not chunk:
                return
            obj = json.loads(chunk.decode('utf-8'))
            if obj.get('type') == 'gameState':
                self.publish_state(obj)

    def publish_state(self, state):
        moves_str = state.get('moves', '')
        status = state.get('status', '')
        board = chess.Board()
        if moves_str:
            for m in moves_str.split():
                try:
                    board.push(chess.Move.from_uci(m))
                except Exception:
                    self.get_logger().warn(f"Illegal move ignored: {m}")
        bmsg = BoardState()
        bmsg.fen = board.fen()
        bmsg.game_id = self._game_id or ''
        bmsg.white_to_move = board.turn
        self._board_pub.publish(bmsg)

        emsg = GameEvent()
        emsg.type = 'gameState'
        emsg.game_id = self._game_id or ''
        emsg.status = status
        self._event_pub.publish(emsg)

        # Publish synthetic opponentMove event when it's opponent's last move
        # (optional) can be inferred by move parity

    def start_game_cb(self, req, resp):
        # Optional: create a challenge. Many bots only accept challenges.
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
