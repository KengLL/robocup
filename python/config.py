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

import json
import math
import os

# ── ZMQ ports ────────────────────────────────────────────────────────────────
VISION_PORT = 9090  # SimNode   → RobotNode + VizNode  (world state)
STRATEGY_PORT = 9091  # StrategyNode → RobotNode        (autonomous targets)
COMMAND_PORT = 9092  # RobotNode → SimNode              (wheel commands)
MANUAL_PORT = 9093  # VizNode   → RobotNode            (manual click targets)

# ── Field geometry (meters) ──────────────────────────────────────────────────
# Default 9 x 6. ROBOCUP_FIELD overrides it for every node that imports this
# file (run_pipeline.py --field sets it for its children): "W H" in metres, or
# a path to robocup_testing's calib/field_pose.json, whose "field" block holds
# the measured size. With the sim field equal to the real table, vision_bridge
# maps camera metres 1:1 instead of stretching the table onto 9 x 6.
DEFAULT_FIELD_W = 9.0
DEFAULT_FIELD_H = 6.0
FIELD_ENV = "ROBOCUP_FIELD"


def parse_field(spec: str) -> tuple[float, float]:
    """'1.93 1.32', '1.93,1.32' or a field_pose.json path -> (W, H) metres."""
    nums = spec.replace(",", " ").split()
    if len(nums) == 2 and not os.path.isfile(spec):
        try:
            w, h = float(nums[0]), float(nums[1])
        except ValueError:
            raise ValueError(f"{FIELD_ENV}: expected 'W H' or a JSON path, got {spec!r}")
    else:
        try:
            with open(spec) as f:
                fld = json.load(f).get("field") or {}
            w, h = float(fld["width"]), float(fld["height"])
        except (OSError, ValueError, KeyError, TypeError) as e:
            raise ValueError(
                f"{FIELD_ENV}: no field width/height in {spec!r} ({e!r}). Re-run "
                + "robocup_testing's `calibrate-field --field W H`, or pass the numbers."
            )
    if w <= 0 or h <= 0:
        raise ValueError(f"{FIELD_ENV}: field size must be positive, got {w} x {h}")
    return w, h


FIELD_W, FIELD_H = (
    parse_field(os.environ[FIELD_ENV])
    if os.environ.get(FIELD_ENV)
    else (DEFAULT_FIELD_W, DEFAULT_FIELD_H)
)
# False when the field isn't 9 x 6: the RL kick policy (trained on 9 x 6) is
# disabled, and vision_bridge expects the tracker's field to match this size.
FIELD_IS_DEFAULT = math.isclose(FIELD_W, DEFAULT_FIELD_W) and math.isclose(FIELD_H, DEFAULT_FIELD_H)
# Robot/ball body geometry shrinks with the field (1.0 on 9 x 6) so a robot covers
# the same share of a small table. Physics (mass, force, damping) is unscaled.
FIELD_SCALE = min(FIELD_W / DEFAULT_FIELD_W, FIELD_H / DEFAULT_FIELD_H)

# Goal mouth height (meters). The opening centered on each short side of the
# field that the ball must cross to score. Previously duplicated between
# simulation_node.py and viz_node.py. Keeps the same fraction of FIELD_H on a
# custom field so it still fits (1.43 m would exceed a 1.32 m table).
# TODO(hardware): real SSL Div-B goals are 1.0 m wide; SSL-EV is 0.8 m.
# Pick one standard for your target league before training.
GOAL_MOUTH_H = 200.0 / 140.0 * (FIELD_H / DEFAULT_FIELD_H)  # ≈ 1.43 m on 9 x 6 (legacy Godot demo value)
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
NUM_HUMAN_CONTROLLERS = 2

# ── Robot hardware (SI units) ────────────────────────────────────────────────
# TODO(hardware): the four values below came from the Godot demo scene
# (140 px/m). They are DIMENSIONALLY correct (meters, kilograms, newtons)
# but NUMERICALLY arbitrary and must be re-measured from the real robot.
ROBOT_RADIUS = 20.0 / 140.0 * FIELD_SCALE  # ≈ 0.143 m on 9 x 6
ROBOT_MASS = 0.8  # kg
WHEEL_DISTANCE = 15.0 / 140.0 * FIELD_SCALE  # ≈ 0.107 m on 9 x 6, radius of wheel placement circle
MOTOR_MAX_FORCE = 200.0 / 140.0  # ≈ 1.429 N per wheel at full command
WHEEL_ANGLES = [0.0, 2 * math.pi / 3, 4 * math.pi / 3]

# ── Ball (SI units) ──────────────────────────────────────────────────────────
# TODO(hardware): real SSL golf ball is 0.0215 m radius / 46 g. The radius
# value below does not match the comment and needs to be reconciled.
BALL_RADIUS = 0.043 * FIELD_SCALE  # meters on 9 x 6 (comment in source claims 43 mm diameter)
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
# The field is always ~900 px wide, whatever its size in metres.
DISPLAY_SCALE = 900.0 / FIELD_W  # pixels per meter on the Pygame window (100 on 9 x 6)
DISPLAY_W = round(FIELD_W * DISPLAY_SCALE)  # 900
DISPLAY_H = round(FIELD_H * DISPLAY_SCALE)  # 600 on 9 x 6

# ── Navigation thresholds ────────────────────────────────────────────────────
ARRIVAL_THRESH = 5.0 / 140.0  # ≈ 0.036 m  (legacy 5 px arrival threshold)

# ── Vision mirror (vision_bridge.py) ─────────────────────────────────────────
# AprilTag id -> sim robot id, or None to auto-assign: each new tag id seen on
# the field takes the lowest free robot. Mirrored robots ignore wheel/kick/dribble
# commands; the camera owns them. Override with --tag-map.
VISION_TAG_MAP = None
# Tags never mirrored, e.g. field-calibration corner tags (0-3 by default) left on
# the table. Override with --ignore-tags.
VISION_IGNORE_TAGS = ()

# Real field origin corner vs sim origin. Flip if goals come out swapped.
# TODO(calibration): set once the real field's origin corner is fixed.
VISION_FLIP_X = False
VISION_FLIP_Y = False

# Age of the last visible sighting (seconds, capture time -> now).
VISION_FRESH_S = 0.15  # younger: camera drives the body
VISION_BALL_COAST_S = 0.35  # younger: sim physics coasts the ball
VISION_ROBOT_COAST_S = 0.5  # younger: robot coasts on its last velocity; older = lost

VISION_BLEND_TAU = 0.10  # seconds, time constant pulling a body onto the measurement
VISION_SNAP_DIST = 0.5  # meters (sim), teleport instead of blending beyond this
VISION_SNAP_ANGLE = math.pi / 2  # rad

# Outlier gate, real-world units. A sighting further than
# max_speed * gap + margin from the last accepted one is rejected.
VISION_BALL_MAX_SPEED = 4.0  # m/s
VISION_ROBOT_MAX_SPEED = 2.0  # m/s
VISION_GATE_MARGIN = 0.05  # meters
VISION_GATE_RELOCK = 3  # consecutive rejects before accepting anyway
VISION_FIELD_MARGIN = 0.05  # meters outside the real field before a sighting is dropped

# Gaps longer than this in a replayed log are squashed to VISION_REPLAY_GAP_CAP.
VISION_REPLAY_GAP_CAP = 1.0  # seconds
VISION_STATS_WINDOW = 90  # frames, rolling window for detection drop rates
