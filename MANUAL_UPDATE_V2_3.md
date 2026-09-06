# K9 Chess V2.3 — manual update

> **Bundle revision r2:** documentation/comments now use the correct Lichess
> bot account name, `K9-chess-bot`. Runtime authentication still comes solely
> from `LICHESS_BOT_TOKEN`.

V2.3 is a small corrective release over V2.2.

## Files that changed

Copy these over the corresponding V2.2 files:

1. `interfaces/action/ComputeChessMove.action`
2. `k9_chess_pkg/k9_chess_pkg/chess_engine_node.py`
3. `k9_chess_pkg/k9_chess_pkg/chess_manager_node.py`

Version/diagnostic metadata also changed:

4. `k9_chess_pkg/k9_chess_pkg/check_setup.py`
5. `k9_chess_pkg/setup.py`
6. `k9_chess_pkg/package.xml`
7. `k9_chess_pkg/VERSION`

The other files are included so this ZIP is also a complete V2.3 source bundle.

## Behaviour fixes

### 1. Terminal stream EOF

A normal final `gameState` (`mate`, `resign`, `draw`, etc.) is detected by
the network thread itself. The subsequent stream EOF is therefore not logged as
`Game stream ... ended; reconnecting`.

### 2. Move event ordering

The event order is now deliberately:

    K9_MOVE_SELECTED
    K9_MOVE_SENT
    K9_MOVE_CONFIRMED
    YOUR_MOVE

`K9_MOVE_SENT` means that the move has been handed to the Lichess transport.
`K9_MOVE_CONFIRMED` remains the authoritative acknowledgement from the Lichess
game stream. If the HTTP submission fails, `MOVE_SEND_ERROR` follows.

### 3. Mate evaluation is retained

`ComputeChessMove.action` now returns:

    bool position_is_mate
    int32 position_mate_in

for the position before K9's selected move, in addition to the already-existing
`resulting_is_mate` / `resulting_mate_in`.

Consequently a `POSITION_EVALUATED` event can now report, for example:

    evaluation_after_valid: false
    is_mate: true
    mate_in: 3

rather than silently losing the mate score.

## Rebuild

Because `ComputeChessMove.action` changed, rebuild `k9_interfaces_pkg` first:

    cd ~/k9_ws

    colcon build --symlink-install --packages-select k9_interfaces_pkg
    source install/setup.bash

    colcon build --symlink-install --packages-select k9_chess_pkg
    source install/setup.bash

If ROS still sees the old action definition, remove only the generated products
for the two affected packages and rebuild:

    rm -rf build/k9_interfaces_pkg install/k9_interfaces_pkg
    rm -rf build/k9_chess_pkg install/k9_chess_pkg

then run the two builds above again.

## Expected live-game evidence

During a forced mate sequence, `POSITION_EVALUATED` should now carry
`is_mate: true` and a non-zero `mate_in` while `K9_MOVE_SELECTED` continues to
describe the resulting position after K9's proposed move.

At game completion there should be no spurious reconnect warning.
