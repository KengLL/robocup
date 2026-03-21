import numpy as np
from geometry import opp_goal
from prediction import predict_intercept_point
from state import RobotState, BallState

def go_to_ball(robot: RobotState, ball: BallState) -> np.ndarray:
    return predict_intercept_point(ball, robot, 0.9, 2.0, 20)


def shoot(color: str) -> np.ndarray:
    return opp_goal(color)


def pass_to(teammate: RobotState) -> np.ndarray:
    return teammate.pos.copy()