# k9_chess/utils/phrases.py
import random

PHRASES = {
    "your_move": [
        "What's your move?", "Your move!", "You're up!", "Your go!",
        "It's your move!", "It's your turn!", "Your turn!", "Your chance now",
        "Over to you", "Your turn now", "Your turn to move"
    ],
    "invalid_move": [
        "Invalid move", "Sorry, invalid move", "Incorrect move",
        "That move is not valid", "You have miscalculated, that is an invalid move",
        "Move not valid", "Improper move", "You are at fault - improper move",
        "Sorry you can't make that move"
    ],
    "check": ["Check", "You are in check", "I have you in check", "You're in check"],
    "winning": [
        "I think I'm winning", "I'm now in front", "This is looking good for me",
        "My position looks strong", "Things are looking negative for you",
        "I am very happy with this game", "It's not looking good for you", "I like winning"
    ],
    "losing": ["You are a very good player", "I'm feeling rather negative about this"],
    "mate_win": ["You should prepare for your end", "It's almost over for you",
                 "The end is near for you", "I will mate soon", "We are near the end of the game"],
    "mate_lose": ["This is not possible", "How can I be losing?", "You are the better player", "This is not logical - I am losing"],
    "draw": ["We are heading for a draw", "The game is looking very even", "This is a well-balanced game", "We are drawing, who will make the winning move?"],
    "instruction": ["I have moved my", "I will move", "I've moved my", "My move is"],
    "takes": ["takes", "captures", "triumphs over", "seizes", "traps", "secures", "gets", "nabs"]
}

def random_msg(key: str) -> str:
    """Return a random message from the dictionary for a given key."""
    options = PHRASES.get(key, [""])
    return random.choice(options)