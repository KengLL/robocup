# RoboCup Python Simulation

A Python port of the Godot-based RoboCup simulation. Replaces the Godot engine with a pure-Python stack: Pymunk for physics, Pygame for visualization, and ZMQ for inter-process messaging.

## Project Overview

The pipeline runs four independent processes that communicate over ZMQ sockets:

```
┌──────────────────────────────────────────────────────────────┐
│                      run_pipeline.py                         │
│              (spawns and monitors all nodes)                 │
└──────────────────────────────────────────────────────────────┘

  simulation_node.py    strategy_node.py    robot_node.py       viz_node.py
  ──────────────────    ────────────────    ─────────────       ───────────
  Pymunk physics        world state ──→     ←── wheel cmds     click → target
  world state ────────→ decide targets ──→  robot control  ──→ Pygame renderer
  (VISION_PORT PUB)     (STRATEGY_PORT PUB) (COMMAND_PORT PUSH) (MANUAL_PORT PUB)
```

- **SimNode** runs the physics world and publishes robot positions/velocities.
- **StrategyNode** subscribes to world state, runs autonomous strategy, and publishes target positions.
- **RobotNode** reads world state, strategy targets, and manual targets, then pushes wheel speed commands.
- **VizNode** renders the field and lets the user set targets by clicking. Press **5** to toggle strategy on/off.

## Key Features

- Three interchangeable robot controllers (selectable at runtime):
  - **PD (Dynamic Inversion)** — closed-form PD control with imposed error dynamics
  - **Time-Optimal (Bang-Bang)** — full acceleration until braking distance, then deceleration
  - **MPC** — greedy rollout over 8 candidate inputs across a 10-step horizon
- Omnidirectional drive with 3-wheel inverse kinematics
- Physics faithful to the original Godot scene (same damping constants, gains, and kinematics)
- Arrival telemetry: prints path length, elapsed time, and average speed on arrival

## Tech Stack

| Library | Version | Purpose |
|---------|---------|---------|
| [Pymunk](http://www.pymunk.org/) | ≥ 6.0 | Rigid-body physics (replaces Godot's physics engine) |
| [Pygame](https://www.pygame.org/) | ≥ 2.0 | Real-time 2-D visualization |
| [PyZMQ](https://pyzmq.readthedocs.io/) | ≥ 24.0 | Inter-process messaging (PUB/SUB, PUSH/PULL) |
| [NumPy](https://numpy.org/) | ≥ 1.21 | Linear algebra for controllers |

Python 3.9+ is supported (uses `from __future__ import annotations` for type hints).

## Folder Structure

```
python/
├── config.py            # All shared constants (ports, field, robot hardware, gains)
├── simulation_node.py   # Pymunk physics engine — publishes world state
├── robot_node.py        # Control loop — reads state, outputs wheel speeds
├── viz_node.py          # Pygame renderer + mouse/keyboard input
├── vision_bridge.py     # Camera tracking → sim coordinates (--vision)
├── run_pipeline.py      # Launcher — spawns all four nodes as subprocesses
└── requirements.txt     # Python dependencies

decision_making/
├── strategy_node.py     # Autonomous strategy
├── state.py             # GameState dataclass + builder
├── geometry.py          # Spatial queries (distances, goal positions)
├── prediction.py        # Ball trajectory prediction + intercept
├── plays/               # Team-level play selection (offensive, defensive, kickoff)
├── tactics/             # Per-robot role assignment (attacker, supporter, defender)
├── skills/              # Low-level actions (navigate, kick, dribble)
├── BT/                  # Behavior tree engine
└── RL/                  # Reinforcement learning (future)
```

## Setup

```bash
# From the python/ directory
pip install -r requirements.txt
```

uv works too, as a local choice. Don't `uv init` / `uv add`: the repo has no `pyproject.toml` on purpose, and `.venv/` is gitignored.

```powershell
# From the repo root, once
uv venv .venv --python 3.12
uv pip install -p .venv -r python/requirements.txt

# Then, from python/
uv run --no-project python run_pipeline.py
```

## Running

```bash
# Start all four nodes (simulation + strategy + robot controller + visualization)
python run_pipeline.py

# Headless mode — skip the Pygame window (e.g. for CI or SSH)
python run_pipeline.py --no-viz

# Skip the strategy node
python run_pipeline.py --no-strategy

# Control the red team instead of blue
python run_pipeline.py --color red

# Mirror the real field from the camera, or replay a recording (see Vision Mirror)
python run_pipeline.py --vision tcp://localhost:5556
python run_pipeline.py --vision path/to/session.jsonl
```

Press **Ctrl+C** to stop all nodes cleanly.

## Development Workflow

### Running a single node manually

Each node can be run in isolation for debugging:

```bash
python simulation_node.py                              # starts physics, publishes on port 9090
python ../decision_making/strategy_node.py --mode zmq  # strategy on port 9091
python robot_node.py                                   # c# connects to sim, reads port 5555, pushes to port 9092
python viz_node.py                                     # # connects to sim, renders, publishes targets on port 9093
```

### Controls (VizNode)

| Input | Action |
|-------|--------|
| Left click on field | Send target to robot |
| `1` | Switch to PD controller |
| `2` | Switch to Time-Optimal controller |
| `3` | Switch to MPC controller |
| `4` | Switch to MANUAL mode (WASD / gamepad) |
| `5` | Toggle autonomous strategy on/off |
| `WASD` | Manual movement (MANUAL mode) |
| `Q` / `E` | Rotation control |
| Gamepad | Left stick: move, Right stick: rotate |

### ZMQ Port Map

| Constant | Port | Direction | Description |
|----------|------|-----------|-------------|
| `VISION_PORT` | 9090 | SimNode → RobotNode, StrategyNode, VizNode | World state (JSON) |
| `STRATEGY_PORT` | 9091 | StrategyNode → RobotNode | Autonomous strategy targets (JSON) |
| `COMMAND_PORT` | 9092 | RobotNode → SimNode | Wheel speed commands (JSON) |
| `MANUAL_PORT` | 9093 | VizNode → RobotNode | Manual targets + strategy toggle (JSON) |

### Message Formats

```jsonc
// World state (VISION_PORT)
{ "t": 1234567890.0, "ball": { "x": 4.5, "y": 3.0, "vx": 0.2, "vy": -0.1 },
  "robots": { "0": { "x": 2.0, "y": 3.0, "vx": 0.0, "vy": 0.0, "angle": 0.0, "omega": 0.0 } } }

// Strategy target (STRATEGY_PORT)
{ "targets": { "0": { "x": 5.2, "y": 3.1 } } }

// Manual target (MANUAL_PORT)
{ "targets": { "0": { "x": 4.5, "y": 3.0, "mode": "2005_INVERSION" } } }

// Strategy toggle (MANUAL_PORT)
{ "strategy_enabled": true }

// Wheel command (COMMAND_PORT)
{ "robot_id": 0, "wheel_speeds": [0.5, -0.3, 0.8] }
```

## Vision Mirror

`--vision` feeds camera tracking from [robocup_testing](https://github.com/ana-jiangR/robocup_testing)'s `track-combined` into SimNode. Tracked objects follow the camera; everything else stays simulated, so a real robot can play against sim robots. `vision_bridge.py` does the work.

### Before you start (once)

1. **Folders.** Commands below assume both repos sit side by side:
   `F:\Cornell\CornellCupRobotics\robocup` and `F:\Cornell\CornellCupRobotics\robocup_testing`. Adjust paths if yours differ.
2. **Sim environment.** See [Setup](#setup). Every sim command runs **from `robocup\python`**. With uv, prefix it with `uv run --no-project python`; with pip, just `python`.
3. **Tracker environment** (live only). In robocup_testing, use the `sim-feed` branch: `--zmq-pub`, `--no-window` and `--mjpg` aren't on main yet.
   ```powershell
   cd F:\Cornell\CornellCupRobotics\robocup_testing
   git checkout sim-feed
   uv sync --all-packages --extra zmq   # --extra zmq pulls in pyzmq for --zmq-pub
   ```
4. **Calibration** (live only). Without it tag and ball positions are off, and the sim drops sightings that land outside the field.
   ```powershell
   uv run calibrate-camera                      # lens, writes calib/intrinsics.json
   uv run calibrate-field --field 1.2 0.8       # your measured field W H in metres
   uv run calibrate-ball --profile mine --radius-mm 20   # only if your ball has no profile yet
   ```
   Profiles live in `calib/ball_color.json`. sim-feed has `mine` and `lab`; `arkart` is only on main (see [Troubleshooting](#troubleshooting)).

### A. Replay a recording (no camera)

```powershell
cd F:\Cornell\CornellCupRobotics\robocup\python
uv run --no-project python run_pipeline.py --vision ..\..\robocup_testing\session.jsonl
```

- Add `--replay-speed 3` to fast-forward. The replay loops at the end.
- `session.jsonl` is about 14 min: tag 4 until ~7.8 min, then tag 0. With auto mapping tag 4 claims robot 0 once it first lands on the field, and tag 0 then takes robot 1. Tag 4 was recorded uncalibrated, so its robot is mostly grey. `--tag-map 0:0,4:1` gives the old fixed layout.
- Logs from either tracker branch replay the same way.

### B. Live (camera, two terminals)

**Terminal 1, tracker:**

```powershell
cd F:\Cornell\CornellCupRobotics\robocup_testing
uv run track-combined --ball-profile mine --tag-size 0.080 --mjpg --zmq-pub "tcp://*:5556"
```

`--ball-profile` is your profile name and `--tag-size` your tag's measured black square in metres. Keep the quotes around `tcp://*:5556`. Leave the window on the first time to check detections; add `--no-window` once you trust it.

**Terminal 2, sim:**

```powershell
cd F:\Cornell\CornellCupRobotics\robocup\python
uv run --no-project python run_pipeline.py --vision tcp://localhost:5556 --ignore-tags 0,1,2,3
```

Each tag the camera sees on the field takes the lowest free robot (robot 0 first) and keeps it for the session; the terminal prints `tag 9 -> robot 0` and the sim window labels the robot `T9`. `--ignore-tags 0,1,2,3` keeps the field-calibration corner tags (ids 0-3 by default) from becoming robots; drop it if they're off the table, or if a robot wears one of those ids. To pin tags to robots instead, pass `--tag-map 9:0,12:1`. Start order doesn't matter; the sim reconnects if the tracker starts later.

Check the strip along the bottom of the sim window: `UP`, ~30 fps, `lat` under ~50 ms, few `cam drops` / `link drops`. Grey = camera isn't currently seeing that object.

Stop with **Ctrl+C** in each terminal.

HTTP fallback, e.g. with main's tracker: `--serve-http 8000` on the tracker, `--vision http://localhost:8000/state` on the sim. Polling misses and repeats frames, so prefer ZMQ.

### C. Record a session

Add `--json-log` to the tracker command. It can run at the same time as live mode:

```powershell
uv run track-combined --ball-profile mine --tag-size 0.080 --mjpg --zmq-pub "tcp://*:5556" --json-log runs\myrun.jsonl
```

One JSON line per frame, appended (a second launch adds a new run to the same file). Replay it with A:

```powershell
uv run --no-project python run_pipeline.py --vision ..\..\robocup_testing\runs\myrun.jsonl
```

robocup_testing does **not** gitignore `*.jsonl` yet (its README says it does), so recordings show up in `git status` there. Don't commit them by accident.

### Flags

Sim (`run_pipeline.py`, passed through to `simulation_node.py`):

| Flag | Meaning |
|------|---------|
| `--vision SRC` | `path.jsonl` replays a log (loops at the end, gaps between runs squashed to 1 s). `tcp://host:5556` is live over ZMQ. `http://host:8000/state` is live over HTTP. Omit for a normal sim game. |
| `--tag-map T:R,...` | AprilTag id → robot id, e.g. `9:0,12:1`. Default `auto`: each new on-field tag takes the lowest free robot. Mapped robots follow the camera and ignore click/WASD/strategy commands; robot 0 is the click/WASD robot. |
| `--ignore-tags IDS` | Tags never mirrored, e.g. `0,1,2,3` for the corner reference tags. |
| `--flip-x`, `--flip-y` | Mirror the camera field if a robot moves opposite to its tag (origin in a different corner). |
| `--replay-speed X` | Replay only. Default 1. |
| `--no-viz`, `--no-strategy` | As usual. |

Tracker (`track-combined`):

| Flag | Meaning |
|------|---------|
| `--zmq-pub "tcp://*:5556"` | Push every frame to the sim (sim-feed branch) |
| `--serve-http PORT` | Serve the latest frame for HTTP polling |
| `--json-log FILE` | Record every frame for replay |
| `--no-window` | Headless, a few fps faster; Ctrl+C quits (sim-feed branch) |
| `--mjpg` | MJPG from the webcam, needed by many USB cams for 720p at 30 fps (sim-feed branch) |
| `--ball-profile NAME` | Ball color profile from `calib/ball_color.json` |
| `--tag-size M` | Measured tag black-square size, metres (default 0.080) |

### Troubleshooting

| Symptom | Fix |
|---------|-----|
| `can't open file ...\robocup\run_pipeline.py` | Run from `robocup\python`, or call `python\run_pipeline.py`. |
| PowerShell: `The '<' operator is reserved` | A placeholder was pasted literally. Use a real value, e.g. `--ball-profile mine`. |
| Unknown ball profile `arkart` on sim-feed | It's only on main. Copy main's profiles without committing: `git show main:calib/ball_color.json \| Set-Content -Encoding utf8 calib/ball_color.json`, and undo with `git checkout -- calib/ball_color.json` before switching branch. Or run `calibrate-ball` on sim-feed. |
| `--zmq-pub` unrecognized, or "needs pyzmq" | Tracker isn't on sim-feed, or run `uv sync --all-packages --extra zmq`. |
| Strip says `DOWN` | Tracker not running, or `--vision` endpoint/port doesn't match `--zmq-pub`. |
| `Address in use` | Another pipeline or tracker is still running. Close it (check Task Manager for stray `python.exe`). |
| Robot doesn't follow the tag | Check the terminal for `tag N -> robot R` (auto) or `tag N seen but not in --tag-map`. No line at all means the tag never landed on the field: recalibrate. A robot parked in a corner is a corner reference tag: add `--ignore-tags 0,1,2,3`. |
| Robot follows but moves the wrong way | `--flip-x` and/or `--flip-y`. |
| A robot stays grey | Tag not seen recently, or its positions land off the field: recalibrate the field, and watch for the tracker's off-plane warning. |
| Camera won't open / dark green picture | Close other apps using it; `uv run unlock-camera` in robocup_testing. |

### How it works

- **Sources:** `http://.../state` (track-combined `--serve-http`, polled at 60 Hz), `tcp://host:port` (ZMQ SUB, CONFLATE), or a `.jsonl` log replayed at recorded speed (`--replay-speed`, loops at EOF).
- **Mapping:** auto by default (`VISION_TAG_MAP = None`): a tag claims a robot on its first visible, on-field sighting and keeps it; `--ignore-tags` / `VISION_IGNORE_TAGS` exclude ids. `--tag-map TAG:ROBOT,...` pins them instead and warns once about unmapped tags. The real field (from the doc's `field`) is scaled onto 9 x 6 m; `--flip-x/--flip-y` (`VISION_FLIP_X/Y`) pick the origin corner.
- **Per-object state**, from the age of the last accepted sighting:

| State | Age | Robot | Ball |
|-------|-----|-------|------|
| fresh | < `VISION_FRESH_S` | kinematic, velocity-servoed onto the dead-reckoned measurement | velocity-servoed |
| coast | < `VISION_*_COAST_S` | keeps last velocity, damped | sim physics |
| lost  | older / never seen | frozen in place | back to sim |

- Sightings are dropped if the tracker marks them not visible or lost, if they fall outside the real field (`VISION_FIELD_MARGIN`), or if they jump further than `max_speed * gap + margin` (outlier gate, relocks after `VISION_GATE_RELOCK` rejects). The tracker's own coasted positions are never used.
- Error above `VISION_SNAP_DIST` teleports instead of blending (first sighting, reacquire).
- Mirrored robots ignore wheel, kick and dribble commands. Goals are edge-triggered and a camera-owned ball is not reset.
- World state gains `source` / `stale` on robots and ball, and a `vision` stats block (fps, latency, transport drops from `seq` gaps, duplicates, rejects, per-object unseen %, and the tracker's own `stats` under `tracker`). VizNode greys stale objects and shows the stats along the bottom of the field.
- Reads both tracker formats: `seq` / `t_capture` / `lost` / `stats` (sim-feed branch) and `frame` / `run_id` (main). A new `run_id` is a tracker restart, not dropped frames. Replay keeps each frame's recorded capture-to-publish delay.

TODO(hardware): scaling a 1.2 m table up to 9 m inflates speeds x7.5. Fine for viewing, wrong for tuning control or RL.

## Deployment Notes

The simulation node is a **drop-in replacement for a real camera system**. To run on physical hardware:

1. Run SimNode with `--vision` (see Vision Mirror); it keeps publishing the same world-state JSON on `VISION_PORT`.
2. `robot_node.py` and `viz_node.py` require no changes.
