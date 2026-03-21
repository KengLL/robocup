import numpy as np
from geometry import distance, opp_goal, our_goal, is_blocking
from prediction import predict_intercept_point
from state import GameState, RobotState, get_obstacles

SUPPORT_DISTANCE = 1.5
SUPPORT_WIDTH = 1.2
FIELD_W = 9.0
FIELD_H = 6.0


def _clamp_to_field(pos: np.ndarray) -> np.ndarray:
    return np.array([
        np.clip(pos[0], 0.3, FIELD_W - 0.3),
        np.clip(pos[1], 0.3, FIELD_H - 0.3),
    ])


def _support_position(ball_pos, goal_pos, side=1):
    to_goal = goal_pos - ball_pos
    dist = np.linalg.norm(to_goal)
    if dist < 1e-6:
        return ball_pos.copy()
    forward = to_goal / dist
    perp = np.array([-forward[1], forward[0]])
    return _clamp_to_field(ball_pos + forward * SUPPORT_DISTANCE + perp * SUPPORT_WIDTH * side)


def _get_side(robot, ball_pos):
    return 1 if robot.pos[1] > ball_pos[1] else -1


def supporter_decide(robot, gamestate, color):
    ball = gamestate.ball
    goal = opp_goal(color)
    side = _get_side(robot, ball.pos)

    ball_holder = next((r for r in gamestate.our_team if r.has_ball), None)
    exclude = [robot.id] + ([ball_holder.id] if ball_holder else [])
    obstacles = get_obstacles(gamestate, exclude_ids=exclude)

    if gamestate.possession == "ours":
        support_pos = _support_position(ball.pos, goal, side)
        if is_blocking(ball.pos, support_pos, obstacles, radius=0.3):
            support_pos = _support_position(ball.pos, goal, side * -1)
        print(f"[Supporter {robot.id}] GET OPEN")
        return support_pos, False

    if gamestate.possession == "loose":
        dist_to_ball = distance(robot.pos, ball.pos)
        if dist_to_ball < 1.0:
            print(f"[Supporter {robot.id}] CHASE LOOSE BALL")
            return _clamp_to_field(predict_intercept_point(ball, robot, 0.9, 2.0, 20)), False
        print(f"[Supporter {robot.id}] ADVANCE")
        return _support_position(ball.pos, goal, side), False

    # opponent has ball — drop back between ball and our goal
    own_goal = our_goal(color)
    to_own_goal = own_goal - ball.pos
    dist = np.linalg.norm(to_own_goal)
    if dist < 1e-6:
        return ball.pos.copy(), False
    direction = to_own_goal / dist
    perp = np.array([-direction[1], direction[0]])
    safe_pos = ball.pos + direction * (dist * 0.5) + perp * SUPPORT_WIDTH * side
    print(f"[Supporter {robot.id}] DROP BACK")
    return _clamp_to_field(safe_pos), False