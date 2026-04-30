from __future__ import annotations

import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

import numpy as np

from geometry import opp_goal, our_goal
from state import GameState, RobotState


CHASE_DIST = 0.95
KICK_DIST = 0.38
DRIVE_THROUGH_DIST = 1.10

CONTEST_MARGIN = 0.18
SHADOW_STANDOFF = 1.00
SHADOW_LATERAL = 0.55


# RL skill - loaded once if available
_rl_skill = None
_rl_enabled = False


def set_rl_enabled(enabled: bool) -> None:
    global _rl_enabled
    _rl_enabled = enabled


def load_rl_skill(checkpoint_path: str) -> bool:
    global _rl_skill
    try:
        from decision_making.skills.rl_kick import RLKickSkill

        _rl_skill = RLKickSkill(checkpoint_path)
        print(f"[AttackerTree] RL kick loaded from {checkpoint_path}")
        return True
    except Exception as e:
        print(f"[AttackerTree] failed to load RL kick: {e}")
        return False


def _angle_to(from_pos, to_pos):
    diff = to_pos - from_pos
    return float(np.arctan2(diff[1], diff[0]))


def _unit_to_goal(ball_pos, goal_pos):
    to_goal = goal_pos - ball_pos
    dist = np.linalg.norm(to_goal)
    if dist < 1e-6:
        return np.array([1.0, 0.0])
    return to_goal / dist


def _drive_through_ball(ball_pos, goal_pos):
    return ball_pos + _unit_to_goal(ball_pos, goal_pos) * DRIVE_THROUGH_DIST


def _closest_opponent(gamestate: GameState):
    opponents = getattr(gamestate, "opponents_team", [])
    if not opponents:
        return None
    return min(opponents, key=lambda r: float(np.linalg.norm(r.pos - gamestate.ball.pos)))


def _should_yield(robot: RobotState, gamestate: GameState) -> bool:
    opp = _closest_opponent(gamestate)
    if opp is None:
        return False

    ball = gamestate.ball
    our_dist = float(np.linalg.norm(robot.pos - ball.pos))
    opp_dist = float(np.linalg.norm(opp.pos - ball.pos))

    # If we already have the ball, keep going. This avoids giving up possession.
    if our_dist < CHASE_DIST:
        return False

    # If the opponent is clearly closer, do not ram into the same point.
    if opp_dist + CONTEST_MARGIN < our_dist:
        return True

    # If it is basically a tie, use robot id as a deterministic tie-breaker so
    # both teams do not choose to chase on the same tick forever.
    if abs(opp_dist - our_dist) <= CONTEST_MARGIN and opp.id < robot.id:
        return True

    return False


def _shadow_ball(robot: RobotState, gamestate: GameState, color: str):
    ball = gamestate.ball
    goal = our_goal(color)
    attack_goal = opp_goal(color)
    to_attack = attack_goal - goal
    n = np.linalg.norm(to_attack)
    if n < 1e-6:
        to_attack = np.array([1.0 if color == "blue" else -1.0, 0.0])
    else:
        to_attack = to_attack / n

    lateral = np.array([0.0, SHADOW_LATERAL if color == "blue" else -SHADOW_LATERAL])
    target = ball.pos - to_attack * SHADOW_STANDOFF + lateral
    return target, _angle_to(robot.pos, ball.pos)


def _rl_target(robot: RobotState, ball, color: str) -> np.ndarray:
    attacks_right = color == "blue"
    assert _rl_skill is not None
    tx, ty = _rl_skill.target(robot, ball, attacks_right=attacks_right)
    return np.array([tx, ty], dtype=float)


def attacker_decide(
    robot: RobotState,
    gamestate: GameState,
    color: str,
) -> tuple[np.ndarray, bool, float | None, bool]:
    """
    Returns (target_pos, kick, target_angle, dribble).

    Behavior:
      - If the opponent is already closer to the ball, shadow instead of ramming.
      - RL off: chase ball, dribble toward goal, kick when close.
      - RL on: chase ball, turn dribbler on, then hand target choice to RL.
    """
    ball = gamestate.ball
    goal = opp_goal(color)
    dist_to_ball = float(np.linalg.norm(robot.pos - ball.pos))
    goal_angle = _angle_to(robot.pos, goal)

    if _should_yield(robot, gamestate):
        target, angle = _shadow_ball(robot, gamestate, color)
        print(f"[Attacker {robot.id}] SHADOW")
        return target, False, angle, False

    # If close enough, just kick. No lineup requirement; this is intentionally
    # simple so the robot does not sit there trying to be perfect.
    if dist_to_ball < KICK_DIST:
        print(f"[Attacker {robot.id}] KICK")
        return goal.copy(), True, _angle_to(ball.pos, goal), False

    # RL mode: first get to the ball. Only then ask RL where to carry/kick.
    if _rl_enabled and _rl_skill is not None:
        if dist_to_ball > CHASE_DIST:
            print(f"[Attacker {robot.id}] RL CHASE")
            return ball.pos.copy(), False, _angle_to(robot.pos, ball.pos), False

        print(f"[Attacker {robot.id}] RL DRIBBLE")
        return _rl_target(robot, ball, color), False, _angle_to(ball.pos, goal), True

    # Non-RL mode: chase, then keep driving through the ball toward goal with
    # dribble on. The target is never the robot's current position, so it should
    # not freeze on top of the ball.
    if dist_to_ball > CHASE_DIST:
        print(f"[Attacker {robot.id}] CHASE")
        return ball.pos.copy(), False, _angle_to(robot.pos, ball.pos), False

    drive_target = _drive_through_ball(ball.pos, goal)
    print(f"[Attacker {robot.id}] DRIBBLE_TO_GOAL")
    return drive_target, False, goal_angle, True
