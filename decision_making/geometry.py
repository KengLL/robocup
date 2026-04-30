import numpy as np
from state import BLUE_GOAL, RED_GOAL, RobotState


# check if there is a clear shot from the robot to the goal
# for attackers
def has_clear_shot(
    robot: RobotState, goal_pos: np.ndarray, opponents: list[RobotState], block_radius: float
) -> bool:
    return not is_blocking(robot.pos, goal_pos, opponents, block_radius)


# check if the ball can be passed
def can_pass_ball(
    passer_robot: RobotState,
    receiver_robot: RobotState,
    opponents: list[RobotState],
    block_radius: float,
) -> bool:
    return not is_blocking(
        passer_robot.pos, receiver_robot.pos, opponents, block_radius
    )


# check if any robot is wihin `radius` of the line segment between `start` and `end`
def is_blocking(
    start: np.ndarray, end: np.ndarray, robots: list[RobotState], radius: float
) -> bool:
    for robot in robots:
        if point_to_line_distance(robot.pos, start, end) < radius:
            return True
    return False


def distance(pos1: np.ndarray, pos2: np.ndarray) -> float:
    return float(np.linalg.norm(pos1 - pos2))


# calculate the distance from a point to a line segment
def point_to_line_distance(
    point: np.ndarray, seg_start: np.ndarray, seg_end: np.ndarray
) -> float:
    # project the point onto the line segment
    v = seg_end - seg_start
    seg_length = np.dot(v, v)
    if seg_length < 1e-9:
        return distance(point, seg_start)
    u = point - seg_start
    proj = np.dot(u, v) / seg_length
    proj = np.clip(proj, 0, 1)
    closest = seg_start + proj * v
    return distance(point, closest)


# Get the goal position for a given team
def our_goal(color: str) -> np.ndarray:
    if color == "blue":
        return BLUE_GOAL.copy()
    elif color == "red":
        return RED_GOAL.copy()
    else:
        raise ValueError("Invalid color")


# Get the opponent's goal position for a given team
def opp_goal(color: str) -> np.ndarray:
    if color == "blue":
        return RED_GOAL.copy()
    elif color == "red":
        return BLUE_GOAL.copy()
    else:
        raise ValueError("Invalid color")


def lerp(a: np.ndarray, b: np.ndarray, t: float) -> np.ndarray:
    return a + (b - a) * t
