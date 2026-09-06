#!/usr/bin/env python3
"""Small deployment check for K9 chess dependencies without exposing tokens."""

from __future__ import annotations

import os
from pathlib import Path
import sys


def _present(name: str) -> str:
    value = os.getenv(name, "")
    return "SET" if value.strip() else "MISSING"


def main() -> None:
    stockfish = Path(
        os.getenv(
            "STOCKFISH_PATH",
            "/home/pi/Stockfish-sf_15/src/stockfish",
        )
    ).expanduser()
    book = Path(
        os.getenv(
            "K9_CHESS_BOOK_PATH",
            "/home/pi/k9-chess-angular/Titans.bin",
        )
    ).expanduser()

    checks = []

    print("K9 chess deployment check")
    print("-------------------------")
    print(f"Phantom username:     {os.getenv('LICHESS_USERNAME', '<config/default hopkira>')}")
    print(f"LICHESS_BOT_TOKEN:    {_present('LICHESS_BOT_TOKEN')}")
    print(f"STOCKFISH_PATH:       {stockfish}")
    print(f"K9_CHESS_BOOK_PATH:   {book}")

    checks.append(bool(os.getenv("LICHESS_BOT_TOKEN", "").strip()))
    checks.append(stockfish.is_file() and os.access(stockfish, os.X_OK))

    print(f"Stockfish executable: {'OK' if checks[-1] else 'MISSING/NOT EXECUTABLE'}")
    print(f"Titans opening book:  {'OK' if book.is_file() else 'NOT FOUND (optional)'}")

    try:
        import chess  # noqa: F401
        print("python-chess:          OK")
        checks.append(True)
    except ImportError:
        print("python-chess:          MISSING")
        checks.append(False)

    try:
        import requests  # noqa: F401
        print("requests:              OK")
        checks.append(True)
    except ImportError:
        print("requests:              MISSING")
        checks.append(False)

    try:
        from k9_interfaces_pkg.action import ComputeChessMove  # noqa: F401
        from k9_interfaces_pkg.msg import ChessEvent, ChessStatus  # noqa: F401
        print("V2.1 chess interfaces:   OK")
        checks.append(True)
    except ImportError as exc:
        print(f"V2.1 chess interfaces:   MISSING ({exc})")
        checks.append(False)

    if all(checks):
        print("\nRequired chess components are present.")
    else:
        print("\nOne or more required components are missing.")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
