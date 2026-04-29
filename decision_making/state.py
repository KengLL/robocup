from dataclasses import dataclass, field
from typing import Any

import numpy as np

DRIBBLER_RANGE = 0.05  # ball within this = "has ball", meters
FIELD_LENGTH = 9.0  # meters
FIELD_WIDTH = 6.0  # meters
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
BLUE_GOAL = np.array([0.0, FIELD_WIDTH / 2])  # [0.0, 3.0]
RED_GOAL = np.array([FIELD_LENGTH, FIELD_WIDTH / 2])  # [9.0, 3.0]

# Goal mouth bounds (mirrors python/config.py — kept here so decision_making
# code does not have to reach into the python/ harness for geometry).
GOAL_MOUTH_H = 200.0 / 140.0  # ≈ 1.43 m
GOAL_Y_MIN = (FIELD_WIDTH - GOAL_MOUTH_H) / 2.0
GOAL_Y_MAX = GOAL_Y_MIN + GOAL_MOUTH_H


# Builder
def build_game_state(raw: dict[str, Any], our_color: str = "blue") -> GameState:
    gamestate = GameState()
    gamestate.timestamp = raw.get("t", 0.0)

    # ball
    ball = raw.get("ball", {})
    vel = np.array([ball.get("vx", 0.0), ball.get("vy", 0.0)])
    gamestate.ball = BallState(
        pos=np.array([ball.get("x", 4.5), ball.get("y", 3.0)]),
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

# returns all robots that are obstacles except for those in exclude_ids
def get_obstacles(gamestate: GameState, exclude_ids: list[int] = None) -> list[RobotState]:
    exclude_ids = exclude_ids or []
    all_robots = gamestate.blue + gamestate.red
    return [r for r in all_robots if r.id not in exclude_ids]