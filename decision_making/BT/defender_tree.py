import numpy as np
from geometry import distance, our_goal, is_blocking
from prediction import predict_intercept_point
from state import GameState, RobotState, get_obstacles

DEFENDER_CHASE_RADIUS = 3.0

def _block_position(ball_pos, goal_pos, offset=0.5):
    to_ball = ball_pos - goal_pos
    dist = np.linalg.norm(to_ball)
    if dist < 1e-6:
        return goal_pos.copy()
    direction = to_ball / dist
    return goal_pos + direction * offset

def defender_decide(robot, gamestate, color):
    ball = gamestate.ball
    goal = our_goal(color)
    dist_to_ball = distance(robot.pos, ball.pos)
    obstacles = get_obstacles(gamestate, exclude_ids=[robot.id])

    if gamestate.possession == "theirs":
        # check if an opponent has a clear path to shoot, if so get in middle line
        shot_is_clear = not is_blocking(ball.pos, goal, obstacles, radius=0.3)
        if shot_is_clear:
            print(f"[Defender {robot.id}] BLOCK SHOT")
            return _block_position(ball.pos, goal, offset=0.6), False
        # shot is already obstructed, hold safe position
        print(f"[Defender {robot.id}] HOLD")
        return _block_position(ball.pos, goal, offset=1.2), False

    if gamestate.possession == "loose" and dist_to_ball < DEFENDER_CHASE_RADIUS:
        print(f"[Defender {robot.id}] INTERCEPT")
        return predict_intercept_point(ball, robot, 0.9, 2.0, 20), False

    print(f"[Defender {robot.id}] HOLD")
    return _block_position(ball.pos, goal, offset=1.2), False