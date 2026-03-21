import numpy as np
from state import BallState, RobotState


# predict where the robot will be in `time_ahead` seconds from now
def predict_ball_position(ball_state: BallState, time_ahead: float) -> np.ndarray:
    return ball_state.pos + ball_state.vel * time_ahead


# find the earliest intercept point where the robot can meet the moving ball.
# at each step, checks if the robot could arrive at the ball's predicted position before the ball gets there.
# returns the intercept point / the ball's current position if no intercept is found.
def predict_intercept_point(
    ball_state: BallState,
    robot_state: RobotState,
    max_robot_speed: float,
    time_ahead: float,
    steps_ahead: int,
) -> np.ndarray:
    if ball_state.speed <= 0.01:
        return ball_state.pos.copy()
    robot_speed = float(np.linalg.norm(robot_state.vel))
    max_robot_speed = max(max_robot_speed, robot_speed)
    for i in range(1, steps_ahead + 1):
        t = (i / steps_ahead) * time_ahead
        future_ball_pos = predict_ball_position(ball_state, t)
        dist_to_future = np.linalg.norm(robot_state.pos - future_ball_pos)
        time_to_future = (
            dist_to_future / max_robot_speed if max_robot_speed > 0 else float("inf")
        )
        if time_to_future <= t:
            return future_ball_pos
    return ball_state.pos.copy()