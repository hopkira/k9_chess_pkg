# K9 Chess Robot

A ROS 2 chess-playing robot framework integrating **Lichess.org**, **Stockfish**, and a behavior tree orchestrator for interactive play. Designed for K9, the conversational robot, to play chess while remaining responsive to speech and other robot states.

---

## 📦 Package Structure


---

## ⚙️ Features

- Connects to Lichess and accepts challenges automatically.
- Maintains current board state in a dedicated **Chess State Node**.
- Computes best moves using Stockfish with optional Polyglot opening book.
- Publishes game events and updates for other nodes to react.
- Sends computed moves back to Lichess via a **Move Sender Service**.
- Behavior Tree orchestrator ensures proper turn-taking and reacts to game events.
- Modular design allows integration with robot conversation, hotword detection, and battery monitoring.

---

## 🛠️ Installation

1. Clone the repository into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone <repo_url> k9_chess
git clone <repo_url> k9_chess_interfaces

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

colcon build
source install/setup.bash
```

