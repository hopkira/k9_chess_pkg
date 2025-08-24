string type        # "challenge" | "gameStart" | "gameEnd" | "gameState" | "opponentMove"
string game_id
string color       # side assigned to us ("white"/"black"), optional
string move_uci    # present for opponentMove
string status      # present for gameState/gameEnd