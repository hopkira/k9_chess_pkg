# K9 Chess Package V2.1

V2.1 combines the useful ROS decomposition from the earlier
`hopkira/k9_chess_pkg` repository with the proven 2025 incoming-challenge
workflow and K9's current central Behaviour Tree architecture.

The important Phantom constraint is explicit: **the Phantom phone app remains
the board gateway and creates the Lichess game**.

See `ARCHITECTURE.md` for the design rationale.

## Runtime model

```text
Physical Phantom board
        ↕ Bluetooth
Phantom phone app
        ↕ Lichess
hopkira  vs  k9_bot
        ↕
chess_manager
        ↕ ComputeChessMove action
chess_engine
        ↕
Titans.bin / Stockfish
```

`hopkira` is a technical account identity. It is **not** the actual person's
name.

The real player name is supplied by the central K9 BT, ideally from existing
conversation/face-recognition context and otherwise by asking:

```text
Who am I playing?
```

The chess package stores that real/display name in `ChessStatus.player_name`.

## Credentials

The manager requires only:

```bash
LICHESS_BOT_TOKEN
```

The historical `LICHESS_PLAYER_TOKEN` is no longer used. The Phantom app owns
the human/player side of game creation.

The fixed Phantom Lichess account is configured in `config/chess.yaml` as:

```yaml
phantom_player_username: "hopkira"
```

`LICHESS_USERNAME=hopkira` remains a valid environment default if the YAML
parameter is removed, but it is not a secret.

## Engine paths

Defaults:

```text
STOCKFISH_PATH=/home/pi/Stockfish-sf_15/src/stockfish
K9_CHESS_BOOK_PATH=/home/pi/k9-chess-angular/Titans.bin
```

The opening book is optional. Stockfish is required.

## Install

Unzip this bundle somewhere convenient, then:

```bash
cd <unzipped-bundle>
./install_v2_1.sh ~/k9_ws
```

If `~/k9_ws/src/k9_chess_pkg` already exists, the installer moves it to a
versioned backup beneath `~/k9_ws/backups/` before installing V2.1.

The installer patches the existing `k9_interfaces_pkg`; it does not replace
the package.

Install Python dependencies in K9's normal ROS Python environment:

```bash
python3 -m pip install python-chess requests
```

Build interfaces first:

```bash
cd ~/k9_ws
colcon build --symlink-install --packages-select k9_interfaces_pkg
source install/setup.bash
```

Then chess:

```bash
colcon build --symlink-install --packages-select k9_chess_pkg
source install/setup.bash
```

## Pre-flight

```bash
ros2 run k9_chess_pkg chess_check
```

It checks the bot token without displaying it, Stockfish, the optional
Titans.bin book, Python dependencies and V2.1 chess interfaces.

## Start the chess nodes

```bash
ros2 launch k9_chess_pkg chess.launch.py
```

Observe durable state:

```bash
ros2 topic echo /chess/status
```

Observe events:

```bash
ros2 topic echo /chess/event
```

## First Phantom test

### 1. Tell the chess subsystem who is physically playing

For example:

```bash
ros2 service call /chess/start_game   k9_interfaces_pkg/srv/StartChessGame   "{player_name: 'Richard'}"
```

Expected state:

```text
WAITING_FOR_CHALLENGE
```

and event:

```text
WAITING_FOR_CHALLENGE
speech_hint:
  Please start the game against me in the Phantom application.
```

### 2. Use the Phantom phone app

Create the game in the Phantom app exactly as before:

```text
Lichess account: hopkira
Opponent:        k9_bot
Colour:          choose in the Phantom app
Time control:    choose in the Phantom app
```

### 3. K9 accepts the incoming challenge

Expected sequence:

```text
WAITING_FOR_CHALLENGE
        ↓
challenge from hopkira
        ↓
CHALLENGE_ACCEPTING
        ↓
CHALLENGE_ACCEPTED
        ↓
gameStart
        ↓
ACTIVE
```

`gameStart` supplies K9's actual assigned colour. `ChessStatus.human_colour`
and `ChessStatus.k9_colour` are blank before that point and authoritative
afterwards.

The real player's display name remains `Richard`; it is never overwritten by
the technical Lichess username `hopkira`.

### 4. Play

The complete Lichess UCI move history is replayed into a fresh `python-chess`
board whenever state is received.

When it is K9's turn:

```text
ENGINE_THINKING
   ↓
weighted Titans.bin move, if available
   otherwise Stockfish
   ↓
K9_MOVE_SELECTED
   ↓
K9_MOVE_SENT
   ↓
K9_MOVE_CONFIRMED
```

`K9_MOVE_SELECTED.speech_hint` contains a deterministic physical-board
instruction such as:

```text
My Knight from g8 to f6.
```

The chess package itself does not speak. The central K9 BT owns `SpeakText`,
eyes, tail and back-panel behaviour.

## Challenge safety

V2.1 accepts a challenge only when **both** are true:

1. `/chess/start_game` has put K9 into `WAITING_FOR_CHALLENGE`; and
2. the challenger username matches `phantom_player_username` (`hopkira`,
   case-insensitively).

All other challenges are declined.

This means an unsolicited internet challenge cannot put K9 into chess mode,
and `hopkira` cannot accidentally start a game while the central BT is doing
something else.

## Control

Suspend move generation while retaining the live game:

```bash
ros2 service call /chess/control   k9_interfaces_pkg/srv/ControlChessGame   "{command: 'SUSPEND'}"
```

Resume:

```bash
ros2 service call /chess/control   k9_interfaces_pkg/srv/ControlChessGame   "{command: 'RESUME'}"
```

Resign:

```bash
ros2 service call /chess/control   k9_interfaces_pkg/srv/ControlChessGame   "{command: 'RESIGN'}"
```

Abort:

```bash
ros2 service call /chess/control   k9_interfaces_pkg/srv/ControlChessGame   "{command: 'ABORT'}"
```

Cancel a waiting/finished/error session and return to IDLE:

```bash
ros2 service call /chess/control   k9_interfaces_pkg/srv/ControlChessGame   "{command: 'RESET'}"
```

Read current state synchronously:

```bash
ros2 service call /chess/get_state   k9_interfaces_pkg/srv/GetChessState   "{}"
```

## Central BT integration

V2.1 deliberately does not introduce another chess BT.

The current K9 BT should ultimately:

1. handle `PLAY_CHESS`;
2. obtain the real player's name from already-established identity context
   when reliable;
3. otherwise ask `Who am I playing?`;
4. call `/chess/start_game` with that display name only;
5. speak the `WAITING_FOR_CHALLENGE` hint;
6. wait for `GAME_STARTED`;
7. map `ChessStatus` onto the existing chess blackboard;
8. route chess event speech hints through the existing priority-aware
   `SpeakText`;
9. drive tail/eyes/back-panel reactions from grounded chess events.

The BT does **not** ask for colour. Colour is selected in the Phantom app and
reported authoritatively by Lichess.
