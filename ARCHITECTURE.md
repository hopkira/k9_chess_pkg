# K9 Chess V2.1 Architecture

## Design objective

K9 has one executive Behaviour Tree. Chess is a subsystem.

The Phantom application remains the physical-board gateway.

```text
                 real human player
                        |
                  Phantom board
                        |
                    Bluetooth
                        |
                 Phantom phone app
                        |
                      Lichess
                 hopkira vs k9_bot
                        |
                        v
                  chess_manager
             +----------------------+
             | bot event stream     |
             | challenge validation |
             | challenge acceptance |
             | game stream          |
             | python-chess board   |
             | move submission      |
             +----------+-----------+
                        |
                 ComputeChessMove
                    ROS action
                        |
                        v
                   chess_engine
             +----------------------+
             | Titans.bin Polyglot  |
             | weighted book choice |
             | Stockfish            |
             | position evaluation  |
             +----------------------+
                        ^
                        |
                  central K9 BT
           setup / lifecycle / speech /
           eyes / tail / back panel
```

## Identity separation

There are two different concepts and V2.1 deliberately keeps them separate.

### Lichess technical identity

```text
Phantom/player account: hopkira
K9 bot account:         k9_bot
```

The manager accepts physical-game challenges only from `hopkira`.

### Real human identity

Examples:

```text
Richard
Alice
James
```

The central BT should reuse a reliably established identity from conversation
or face recognition. If none exists it asks the player.

That display name is supplied in `StartChessGame.player_name` and retained in
`ChessStatus.player_name`. A `gameStart` event from Lichess never replaces it
with `hopkira`.

## Game setup state machine

```text
IDLE
  |
  | StartChessGame(player_name)
  v
WAITING_FOR_CHALLENGE
  |
  | incoming challenge from hopkira
  v
STARTING
  |
  | accept + Lichess gameStart
  v
ACTIVE
  |
  +--> SUSPENDED --> ACTIVE
  |
  +--> FINISHED
```

Unexpected challengers are declined.

A challenge from `hopkira` is also declined unless the manager has first been
armed through `StartChessGame`. This preserves central-BT executive control.

## Colour

The BT never asks for a colour.

Colour is selected in the Phantom app and learned from the Lichess `gameStart`
event. The manager derives the human colour as the opposite of K9's assigned
colour and publishes both in `ChessStatus`.

## What V2.1 keeps

From the historical standalone K9 implementation:

- incoming Lichess challenge/event-stream workflow;
- full board reconstruction from the complete UCI move history;
- Lichess as the transport layer to the Phantom ecosystem;
- Titans.bin and Stockfish.

From the earlier ROS chess repository:

- separate Game Manager and Chess Engine responsibilities;
- `ComputeChessMove` ROS action;
- weighted Polyglot opening-book choice;
- structured typed status/events;
- one active game at a time.

From V2.0:

- threaded NDJSON HTTP streams rather than blocking ROS callbacks;
- central BT as the only executive;
- manager submits authenticated moves directly;
- no separate Move Sender, Chess State node or chess BT;
- deterministic speech hints rather than direct robot hardware calls.

## ROS interfaces

### `/chess/start_game` — `StartChessGame`

Request:

```text
string player_name
```

It does not create a Lichess game. It arms the manager and transitions to
`WAITING_FOR_CHALLENGE`.

### `/chess/status` — `ChessStatus`

Important identity/colour fields:

```text
player_name
human_colour
k9_colour
```

`human_colour` and `k9_colour` remain blank until `gameStart`.

### `/chess/event` — `ChessEvent`

Important setup events:

```text
WAITING_FOR_CHALLENGE
CHALLENGE_ACCEPTING
CHALLENGE_ACCEPTED
CHALLENGE_DECLINED
GAME_STARTED
```

Normal game events include:

```text
HUMAN_MOVE
ENGINE_THINKING
POSITION_EVALUATED
K9_MOVE_SELECTED
K9_MOVE_SENT
K9_MOVE_CONFIRMED
YOUR_MOVE
GAME_SUSPENDED
GAME_RESUMED
GAME_FINISHED
ENGINE_ERROR
MOVE_SEND_ERROR
BOARD_RESYNC
```

### `/chess/compute_move` — `ComputeChessMove`

The manager sends the FEN and K9 colour. The engine tries a weighted Polyglot
book move first and falls back to Stockfish.

All evaluation values are from K9's point of view.

### `/chess/control`

Commands:

```text
SUSPEND
RESUME
RESIGN
ABORT
RESET
```

### `/chess/get_state`

Returns the current durable `ChessStatus`.

## Credentials

V2.1 requires `LICHESS_BOT_TOKEN`.

`LICHESS_PLAYER_TOKEN` was required only by the discarded K9-created-game
approach and is no longer part of the runtime architecture.
