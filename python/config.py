"""
Shared configuration constants for the RoboCup pipeline.

All physical quantities are in SI units (meters, seconds, kilograms,
newtons, radians) unless a comment explicitly says otherwise. The two
exceptions are DISPLAY_* values (screen pixels for the Pygame window)
and port numbers.

IMPORTANT: several of the robot-hardware values below were inherited
from the original Godot demo scene (test_iterative.tscn), which used a
coordinate system of 140 px per meter. They have been converted to SI
but NOT re-measured against real hardware — grep for `TODO(hardware)`
for the list of values that must be calibrated before sim-to-real
transfer.
"""

import math

# ── ZMQ ports ────────────────────────────────────────────────────────────────
VISION_PORT = 9090  # SimNode   → RobotNode + VizNode  (world state)
STRATEGY_PORT = 9091  # StrategyNode → RobotNode        (autonomous targets)
COMMAND_PORT = 9092  # RobotNode → SimNode              (wheel commands)
MANUAL_PORT = 9093  # VizNode   → RobotNode            (manual click targets)

# ── Field geometry (meters) ──────────────────────────────────────────────────
FIELD_W = 9.0
FIELD_H = 6.0

# Goal mouth height (meters). The opening centered on each short side of the
# field that the ball must cross to score. Previously duplicated between
# simulation_node.py and viz_node.py.
# TODO(hardware): real SSL Div-B goals are 1.0 m wide; SSL-EV is 0.8 m.
# Pick one standard for your target league before training.
GOAL_MOUTH_H = 200.0 / 140.0  # ≈ 1.43 m (legacy Godot demo value)
GOAL_Y_MIN = (FIELD_H - GOAL_MOUTH_H) / 2.0
GOAL_Y_MAX = GOAL_Y_MIN + GOAL_MOUTH_H

# ── Team composition ─────────────────────────────────────────────────────────
# 3v3. IDs 0..TEAM_BLUE_SIZE-1 are blue; remaining ids up to NUM_ROBOTS are red.
TEAM_BLUE_SIZE = 3
TEAM_RED_SIZE = 3
NUM_ROBOTS = TEAM_BLUE_SIZE + TEAM_RED_SIZE
TEAM_S = TEAM_BLUE_SIZE  # legacy alias, kept for any external readers

# Human controllers drive blue robots 0..NUM_HUMAN_CONTROLLERS-1; the rest of
# blue is AI. Clamped at runtime by the joysticks actually plugged in.
NUM_HUMAN_CONTROLLERS = 0

# ── Robot hardware (SI units) ────────────────────────────────────────────────
# TODO(hardware): the four values below came from the Godot demo scene
# (140 px/m). They are DIMENSIONALLY correct (meters, kilograms, newtons)
# but NUMERICALLY arbitrary and must be re-measured from the real robot.
ROBOT_RADIUS = 20.0 / 140.0  # ≈ 0.143 m
ROBOT_MASS = 0.8  # kg
WHEEL_DISTANCE = 15.0 / 140.0  # ≈ 0.107 m, radius of wheel placement circle
MOTOR_MAX_FORCE = 200.0 / 140.0  # ≈ 1.429 N per wheel at full command
WHEEL_ANGLES = [0.0, 2 * math.pi / 3, 4 * math.pi / 3]

# ── Ball (SI units) ──────────────────────────────────────────────────────────
# TODO(hardware): real SSL golf ball is 0.0215 m radius / 46 g. The radius
# value below does not match the comment and needs to be reconciled.
BALL_RADIUS = 0.043  # meters (comment in source claims 43 mm diameter)
BALL_MASS = 0.046  # kg (46 g, SSL standard)
BALL_DAMP = 0.5  # per-second manual velocity damping coefficient

# ── Physics damping (per-second coefficients) ────────────────────────────────
# TODO(hardware): these are lumped Godot-style damping, not real rolling
# friction. Replace with a measured coastdown curve for the real robot.
LINEAR_DAMP = 3.0
ANGULAR_DAMP = 3.0

# ── Low-level controller gains (robot_node.py) ───────────────────────────────
KP = 15.0
KD = 5.0

# MPC rollout horizon (robot_node.py greedy MPC controller)
MPC_HORIZON = 10
MPC_DT = 0.016

# ── Simulation timing ────────────────────────────────────────────────────────
FPS = 60
DT = 1.0 / FPS

# ── Display (pixels — viz_node only) ─────────────────────────────────────────
DISPLAY_SCALE = 100.0  # pixels per meter on the Pygame window
DISPLAY_W = int(FIELD_W * DISPLAY_SCALE)  # 900
DISPLAY_H = int(FIELD_H * DISPLAY_SCALE)  # 600

# ── Navigation thresholds ────────────────────────────────────────────────────
ARRIVAL_THRESH = 5.0 / 140.0  # ≈ 0.036 m  (legacy 5 px arrival threshold)
