# RoboCup Python Simulation

A Python port of the Godot-based RoboCup simulation. Replaces the Godot engine with a pure-Python stack: Pymunk for physics, Pygame for visualization, and ZMQ for inter-process messaging.

## Project Overview

The pipeline runs three independent processes that communicate over ZMQ sockets:

```
┌──────────────────────────────────────────────────────────────┐
│                      run_pipeline.py                         │
│              (spawns and monitors all nodes)                 │
└──────────────────────────────────────────────────────────────┘

  simulation_node.py          robot_node.py          viz_node.py
  ──────────────────          ─────────────          ───────────
  Pymunk physics          ←── wheel commands     click → target
  world state ──────────────→ robot control  ──→ Pygame renderer
  (VISION_PORT PUB)           (MANUAL_PORT SUB)  (VISION_PORT SUB)
```

- **SimNode** runs the physics world and publishes robot positions/velocities.
- **RobotNode** reads world state and manual targets, then pushes wheel speed commands.
- **VizNode** renders the field and lets the user set targets by clicking.

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

Python 3.10+ is required (uses `X | Y` union type hints).

## Folder Structure

```
python/
├── config.py            # All shared constants (ports, field, robot hardware, gains)
├── simulation_node.py   # Pymunk physics engine — publishes world state
├── robot_node.py        # Control loop — reads state, outputs wheel speeds
├── viz_node.py          # Pygame renderer + mouse/keyboard input
├── run_pipeline.py      # Launcher — spawns all three nodes as subprocesses
└── requirements.txt     # Python dependencies
```

## Setup

```bash
# From the python/ directory
pip install -r requirements.txt
```

## Running

```bash
# Start all three nodes (simulation + robot controller + visualization)
python run_pipeline.py

# Headless mode — skip the Pygame window (e.g. for CI or SSH)
python run_pipeline.py --no-viz
```

Press **Ctrl+C** to stop all nodes cleanly.

## Development Workflow

### Running a single node manually

Each node can be run in isolation for debugging:

```bash
python simulation_node.py   # starts physics, publishes on port 5555
python robot_node.py        # connects to sim, reads port 5555, pushes to 5557
python viz_node.py          # connects to sim, renders, publishes targets on 5558
```

### Controls (VizNode)

| Input | Action |
|-------|--------|
| Left click on field | Send target to robot 0 |
| `1` | Switch to PD controller |
| `2` | Switch to Time-Optimal controller |
| `3` | Switch to MPC controller |

### ZMQ Port Map

| Constant | Port | Direction | Description |
|----------|------|-----------|-------------|
| `VISION_PORT` | 5555 | SimNode → RobotNode, VizNode | World state (JSON) |
| `STRATEGY_PORT` | 5556 | — | Reserved for autonomous strategy node |
| `COMMAND_PORT` | 5557 | RobotNode → SimNode | Wheel speed commands (JSON) |
| `MANUAL_PORT` | 5558 | VizNode → RobotNode | Manual click targets (JSON) |

### Message Formats

```jsonc
// World state (VISION_PORT)
{ "t": 1234567890.0, "robots": { "0": { "x": 2.0, "y": 3.0, "vx": 0.0, "vy": 0.0, "angle": 0.0, "omega": 0.0 } } }

// Manual target (MANUAL_PORT)
{ "targets": { "0": { "x": 4.5, "y": 3.0, "mode": "2005_INVERSION" } } }

// Wheel command (COMMAND_PORT)
{ "robot_id": 0, "wheel_speeds": [0.5, -0.3, 0.8] }
```

## Deployment Notes

The simulation node is a **drop-in replacement for a real camera system**. To run on physical hardware:

1. Replace `simulation_node.py` with a vision node that reads AprilTag poses (or another localisation system).
2. The vision node must publish the same world-state JSON format on `VISION_PORT`.
3. `robot_node.py` and `viz_node.py` require no changes.
