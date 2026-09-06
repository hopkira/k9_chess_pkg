"""Deterministic chess wording for immediate/fallback behaviour.

The central K9 BT/conversation layer may replace these hints with more
characterful dialogue.  These functions never decide chess facts.
"""

from __future__ import annotations

import chess


PIECE_NAMES = {
    chess.PAWN: "Pawn",
    chess.KNIGHT: "Knight",
    chess.BISHOP: "Bishop",
    chess.ROOK: "Rook",
    chess.QUEEN: "Queen",
    chess.KING: "King",
}


def piece_name(piece_type: int | None) -> str:
    if piece_type is None:
        return ""
    return PIECE_NAMES.get(piece_type, "Piece")


def move_instruction(
    piece: str,
    from_square: str,
    to_square: str,
    captured_piece: str = "",
    gives_check: bool = False,
    gives_mate: bool = False,
) -> str:
    if captured_piece:
        text = (
            f"My {piece} from {from_square} takes your "
            f"{captured_piece} on {to_square}."
        )
    else:
        text = f"My {piece} from {from_square} to {to_square}."

    if gives_mate:
        return text + " Checkmate."
    if gives_check:
        return text + " Check."
    return text


def game_started(human_colour: str, k9_colour: str) -> str:
    return (
        f"Affirmative. You are playing {human_colour.lower()}. "
        f"I will play {k9_colour.lower()}."
    )
