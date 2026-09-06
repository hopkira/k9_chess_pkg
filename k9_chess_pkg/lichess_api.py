"""Small, explicit Lichess HTTP client for K9 chess.

Only the endpoints needed by the K9/Phantom workflow are wrapped here.  This
keeps the physical-board transport independent of the much larger lichess-bot
framework while retaining its useful separation of concerns.
"""

from __future__ import annotations

import json
from typing import Iterator, Optional
from urllib.parse import urljoin

import requests


class LichessError(RuntimeError):
    """Base error raised for Lichess HTTP/stream failures."""


class LichessRateLimitError(LichessError):
    """Raised when Lichess returns HTTP 429."""


class LichessAPI:
    """Minimal authenticated Lichess API adapter."""

    def __init__(
        self,
        token: str,
        base_url: str = "https://lichess.org/api/",
    ) -> None:
        token = (token or "").strip()
        if not token:
            raise ValueError("A Lichess OAuth token is required")

        self.base_url = base_url.rstrip("/") + "/"
        self.headers = {
            "Authorization": f"Bearer {token}",
            "Accept": "application/json",
        }
        self.session = requests.Session()
        self.session.headers.update(self.headers)

    def close(self) -> None:
        self.session.close()

    def _url(self, path: str) -> str:
        return urljoin(self.base_url, path.lstrip("/"))

    @staticmethod
    def _check(response: requests.Response) -> None:
        if response.status_code == 429:
            raise LichessRateLimitError(
                "Lichess API rate limit reached (HTTP 429)"
            )

        if not response.ok:
            body = response.text.strip()
            raise LichessError(
                f"Lichess HTTP {response.status_code}: {body}"
            )

    def create_challenge(
        self,
        username: str,
        params: dict,
    ) -> dict:
        response = self.session.post(
            self._url(f"challenge/{username}"),
            data=params,
            timeout=20,
        )
        self._check(response)
        return response.json()

    def accept_challenge(self, challenge_id: str) -> None:
        response = self.session.post(
            self._url(f"challenge/{challenge_id}/accept"),
            timeout=20,
        )
        self._check(response)

    def decline_challenge(
        self,
        challenge_id: str,
        reason: str = "generic",
    ) -> None:
        response = self.session.post(
            self._url(f"challenge/{challenge_id}/decline"),
            data={"reason": reason},
            timeout=20,
        )
        self._check(response)

    def event_stream(self) -> requests.Response:
        response = self.session.get(
            self._url("stream/event"),
            stream=True,
            timeout=(15, 90),
        )
        self._check(response)
        return response

    def game_stream(self, game_id: str) -> requests.Response:
        response = self.session.get(
            self._url(f"bot/game/stream/{game_id}"),
            stream=True,
            timeout=(15, 90),
        )
        self._check(response)
        return response

    def make_move(self, game_id: str, move_uci: str) -> None:
        response = self.session.post(
            self._url(f"bot/game/{game_id}/move/{move_uci}"),
            timeout=20,
        )
        self._check(response)

    def chat(
        self,
        game_id: str,
        text: str,
        room: str = "player",
    ) -> None:
        response = self.session.post(
            self._url(f"bot/game/{game_id}/chat"),
            data={"room": room, "text": text},
            timeout=20,
        )
        self._check(response)

    def resign(self, game_id: str) -> None:
        response = self.session.post(
            self._url(f"bot/game/{game_id}/resign"),
            timeout=20,
        )
        self._check(response)

    def abort(self, game_id: str) -> None:
        response = self.session.post(
            self._url(f"bot/game/{game_id}/abort"),
            timeout=20,
        )
        self._check(response)

    @staticmethod
    def iter_json_lines(
        response: requests.Response,
    ) -> Iterator[dict]:
        """Yield non-empty objects from a Lichess NDJSON stream."""
        for raw_line in response.iter_lines():
            if not raw_line:
                continue

            try:
                yield json.loads(raw_line.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise LichessError(
                    f"Invalid JSON from Lichess stream: {exc}"
                ) from exc

    @staticmethod
    def challenge_id(response: dict) -> Optional[str]:
        """Extract a challenge/game id from historical/current responses."""
        game = response.get("game")
        if isinstance(game, dict) and game.get("id"):
            return str(game["id"])

        challenge = response.get("challenge")
        if isinstance(challenge, dict) and challenge.get("id"):
            return str(challenge["id"])

        if response.get("id"):
            return str(response["id"])

        return None

    @staticmethod
    def response_is_immediate_game(response: dict) -> bool:
        game = response.get("game")
        return isinstance(game, dict) and bool(game.get("id"))
