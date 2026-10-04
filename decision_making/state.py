import os
import sys
from dataclasses import dataclass, field
from typing import Any

import numpy as np

# Field geometry comes from python/config.py so a custom --field reaches strategy.
_PYTHON_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "python")
if _PYTHON_DIR not in sys.path:
    sys.path.insert(0, _PYTHON_DIR)

from config import FIELD_H, FIELD_SCALE, FIELD_W, GOAL_MOUTH_H, GOAL_Y_MAX, GOAL_Y_MIN  # noqa: E402, F401

DRIBBLER_RANGE = 0.05 * FIELD_SCALE  # ball within this = "has ball", meters (9 x 6 value)
FIELD_LENGTH = FIELD_W  # meters, along x (9 by default)
FIELD_WIDTH = FIELD_H  # meters, along y (6 by default)
# 3v3. Robot ids 0..TEAM_BLUE_SIZE-1 are blue;
# ids TEAM_BLUE_SIZE..TEAM_BLUE_SIZE+TEAM_RED_SIZE-1 are red.
TEAM_BLUE_SIZE = 3
TEAM_RED_SIZE = 3


@dataclass
class BallState:
    pos: np.ndarray = field(default_factory=lambda: np.zeros(2))  # [x, y] meters
    vel: np.ndarray = field(default_factory=lambda: np.zeros(2))  # [vx, vy] meters/sec
    speed: float = 0.0  # meters/sec


@dataclass
class RobotState:
    id: int = 0
    pos: np.ndarray = field(default_factory=lambda: np.zeros(2))  # [x, y] meters
    vel: np.ndarray = field(default_factory=lambda: np.zeros(2))  # [vx, vy] meters/sec
    angle: float = 0.0  # radians, 0 = facing +x
    omega: float = 0.0  # angular velocity, rad/sec
    has_ball: bool = False  # is ball within dribbler range


@dataclass
class GameState:
    timestamp: float = 0  # time.time() from sim

    # Ball
    ball: BallState = field(default_factory=BallState)

    # Teams — indices match robot IDs
    blue: list[RobotState] = field(default_factory=list)  # 3 robots
    red: list[RobotState] = field(default_factory=list)  # 3 robots

    # Computed once per frame
    our_team: list[RobotState] = field(
        default_factory=list
    )  # alias for whichever team we control
    opponents_team: list[RobotState] = field(default_factory=list)  # opponents
    possession: str = "loose"  # "ours", "theirs", "loose"
    game_phase: str = "play"  # "play", "kickoff", "freekick", "halt"


# Goals (Fixed Positions)
BLUE_GOAL = np.array([0.0, FIELD_WIDTH / 2])  # [0.0, 3.0] on 9 x 6
RED_GOAL = np.array([FIELD_LENGTH, FIELD_WIDTH / 2])  # [9.0, 3.0] on 9 x 6

# Goal mouth bounds (GOAL_MOUTH_H, GOAL_Y_MIN, GOAL_Y_MAX) are imported from config.


# Builder
def build_game_state(raw: dict[str, Any], our_color: str = "blue") -> GameState:
    gamestate = GameState()
    gamestate.timestamp = raw.get("t", 0.0)

    # ball
    ball = raw.get("ball", {})
    vel = np.array([ball.get("vx", 0.0), ball.get("vy", 0.0)])
    gamestate.ball = BallState(
        pos=np.array([ball.get("x", FIELD_LENGTH / 2), ball.get("y", FIELD_WIDTH / 2)]),
        vel=vel,
        speed=float(np.linalg.norm(vel)),
    )

    # robots
    robots_raw = raw.get("robots", {})
    for i in range(TEAM_BLUE_SIZE):
        blue_robot = robots_raw.get(str(i), {})
        gamestate.blue.append(_parse_robot(i, blue_robot))
    for i in range(TEAM_RED_SIZE):
        red_robot = robots_raw.get(str(i + TEAM_BLUE_SIZE), {})
        gamestate.red.append(_parse_robot(i + TEAM_BLUE_SIZE, red_robot))

    # Check the closest robot within the dribble range
    _compute_has_ball(gamestate)

    # Team Aliases
    if our_color == "blue":
        gamestate.our_team = gamestate.blue
        gamestate.opponents_team = gamestate.red
    else:
        gamestate.our_team = gamestate.red
        gamestate.opponents_team = gamestate.blue

    # Possession of ball
    if any(robot.has_ball for robot in gamestate.our_team):
        gamestate.possession = "ours"
    elif any(robot.has_ball for robot in gamestate.opponents_team):
        gamestate.possession = "theirs"
    else:
        gamestate.possession = "loose"

    return gamestate


# Helper
def _parse_robot(rid: int, raw: dict[str, Any]) -> RobotState:
    return RobotState(
        id=rid,
        pos=np.array([raw.get("x", 0.0), raw.get("y", 0.0)]),
        vel=np.array([raw.get("vx", 0.0), raw.get("vy", 0.0)]),
        angle=raw.get("angle", 0.0),
        omega=raw.get("omega", 0.0),
    )


def _compute_has_ball(gamestate: GameState) -> None:
    best_robot = None
    best_dist = DRIBBLER_RANGE

    for robot in gamestate.blue + gamestate.red:
        dist = float(np.linalg.norm(gamestate.ball.pos - robot.pos))
        if dist < best_dist:
            best_robot = robot
            best_dist = dist

    if best_robot is not None:
        best_robot.has_ball = True
