# V2.3 corrective changes

- suppresses false game-stream reconnect warnings after a terminal state;
- enforces `K9_MOVE_SELECTED -> K9_MOVE_SENT -> K9_MOVE_CONFIRMED -> YOUR_MOVE`;
- carries current-position forced-mate metadata through `ComputeChessMove`.

See `MANUAL_UPDATE_V2_3.md` for the three functional files to copy manually.

# K9 Chess V2.3 bundle

V2.3 supersedes V2.1.

The key Phantom workflow correction is:

- the Phantom phone app creates every physical game;
- the fixed Lichess player account is `hopkira`;
- the fixed opponent is K9's bot account (`K9-chess-bot`);
- K9 does not create the Lichess game;
- `/chess/start_game` only records the real human player's display name and
  arms the manager to accept the next `hopkira` challenge;
- Lichess `gameStart` supplies the actual colours;
- `LICHESS_PLAYER_TOKEN` is no longer required.

The bundle contains:

- `k9_chess_pkg/` — ROS 2 package with `chess_manager` and `chess_engine`.
- `interfaces/` — typed chess messages/services/action for the existing
  `k9_interfaces_pkg`.
- `tools/apply_interfaces_patch.py` — idempotent interface patcher.
- `install_v2_3.sh` — installs the package and patches `k9_interfaces_pkg`
  without replacing its existing interfaces.

Install source files:

```bash
./install_v2_3.sh ~/k9_ws
```

Then follow `k9_chess_pkg/README.md`.

The Lichess bot token is deliberately NOT included. Replace the previously
exposed token before testing; do not paste the replacement into ChatGPT.

## V2.3 Python interpreter fix

V2.3 replaces setuptools-generated Python console entry points with small shell
launchers. This avoids a generated `#!/usr/bin/python3` shebang bypassing the
active K9 virtual environment. `ros2 run k9_chess_pkg ...` now resolves
`python3` from the current `PATH`. You may explicitly override it with:

```bash
export K9_PYTHON=/home/hopkira/k9_venv/bin/python3
```

`chess_check` prints the interpreter it actually uses.
