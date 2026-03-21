import numpy as np
from geometry import distance, our_goal, is_blocking
from prediction import predict_intercept_point
from state import GameState, RobotState, get_obstacles
from BT.conditions import avoid_teammates

DEFENDER_CHASE_RADIUS = 3.0
DEFENDER_GOAL_RADIUS = 2.0  # max distance defender will stray from own goal

def _block_position(ball_pos, goal_pos, offset=0.5):
    to_ball = ball_pos - goal_pos
    dist = np.linalg.norm(to_ball)
    if dist < 1e-6:
        return goal_pos.copy()
    direction = to_ball / dist
    return goal_pos + direction * offset

#keep posiiton within defender goal radius
def _clamp_to_goal_zone(pos, goal_pos):
    diff = pos - goal_pos
    dist = np.linalg.norm(diff)
    if dist > DEFENDER_GOAL_RADIUS:
        pos = goal_pos + (diff / dist) * DEFENDER_GOAL_RADIUS
    return pos

def defender_decide(robot, gamestate, color):
    ball = gamestate.ball
    goal = our_goal(color)
    dist_to_ball = distance(robot.pos, ball.pos)
    obstacles = get_obstacles(gamestate, exclude_ids=[robot.id])

    if gamestate.possession == "theirs":
        shot_is_clear = not is_blocking(ball.pos, goal, obstacles, radius=0.3)
        if shot_is_clear:
            pos = _clamp_to_goal_zone(_block_position(ball.pos, goal, offset=0.6), goal)
            return avoid_teammates(pos, robot, gamestate), False
        pos = _clamp_to_goal_zone(_block_position(ball.pos, goal, offset=1.2), goal)
        return avoid_teammates(pos, robot, gamestate), False

    if gamestate.possession == "loose" and dist_to_ball < DEFENDER_CHASE_RADIUS:
        pos = _clamp_to_goal_zone(predict_intercept_point(ball, robot, 0.9, 2.0, 20), goal)
        return avoid_teammates(pos, robot, gamestate), False

    pos = _clamp_to_goal_zone(_block_position(ball.pos, goal, offset=1.2), goal)
    return avoid_teammates(pos, robot, gamestate), False