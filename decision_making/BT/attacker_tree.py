from __future__ import annotations
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from geometry import opp_goal
from state import GameState, RobotState
from BT.conditions import best_shot_target, teammate_open, MIN_ROBOT_SEPARATION

KICK_DIST     = 0.20
BEHIND_DIST   = 0.3
BEHIND_THRESH = 0.35
JUST_KICK_DIST = 0.25
DRIBBLE_ENGAGE_DIST = 1   # start dribbling when this close to ball
DRIBBLE_CARRY_DIST  = 10   # max distance to carry ball before shooting

# per-robot state
_states = {}

# RL skill — loaded once if available
_rl_skill = None
_rl_enabled = False

def set_rl_enabled(enabled: bool) -> None:
    global _rl_enabled
    _rl_enabled = enabled

def load_rl_skill(checkpoint_path: str) -> bool:
    global _rl_skill
    try:
        from decision_making.rl.rl_kick import RLKickSkill
        _rl_skill = RLKickSkill(checkpoint_path)
        print(f"[AttackerTree] RL kick loaded from {checkpoint_path}")
        return True
    except Exception as e:
        print(f"[AttackerTree] failed to load RL kick: {e}")
        return False

def _get_state(robot_id):
    if robot_id not in _states:
        _states[robot_id] = {
            "mode": "CHASE",
            "shot_target": None,
            "pass_target": None,
            "dribbling": False,
            "carry_start": None,
        }
    return _states[robot_id]

def _behind_ball(ball_pos, shoot_target):
    to_target = shoot_target - ball_pos
    dist = np.linalg.norm(to_target)
    if dist < 1e-6:
        return ball_pos + np.array([-BEHIND_DIST, 0])
    return ball_pos - (to_target / dist) * BEHIND_DIST

def _angle_to(from_pos, to_pos):
    diff = to_pos - from_pos
    return float(np.arctan2(diff[1], diff[0]))

def _approach_ok(robot_pos, ball_pos, goal_pos):
    to_ball = ball_pos - robot_pos
    to_goal = goal_pos - ball_pos
    d1, d2 = np.linalg.norm(to_ball), np.linalg.norm(to_goal)
    if d1 < 1e-6 or d2 < 1e-6:
        return False
    return float(np.dot(to_ball/d1, to_goal/d2)) > 0.4

def attacker_decide(
    robot: RobotState,
    gamestate: GameState,
    color: str,
) -> tuple[np.ndarray, bool, float | None, bool]:
    """
    Returns (target_pos, kick, target_angle, dribble).
    dribble=True tells robot_node to activate the dribbler this tick.
    """
    ball = gamestate.ball
    goal = opp_goal(color)
    opponents = gamestate.opponents_team
    teammates = [r for r in gamestate.our_team if r.id != robot.id]
    dist_to_ball = float(np.linalg.norm(robot.pos - ball.pos))
    state = _get_state(robot.id)

    # ── RL kick mode — hand off entirely to the RL policy ───────────────────
    if _rl_enabled and _rl_skill is not None:
        attacks_right = (color == "blue")
        tx, ty = _rl_skill.target(robot, ball, attacks_right=attacks_right)
        target = np.array([tx, ty])
        # kick when very close and lined up
        kick = dist_to_ball < KICK_DIST and _approach_ok(robot.pos, ball.pos, goal)
        angle = _angle_to(ball.pos, goal) if dist_to_ball < 0.5 else None
        print(f"[Attacker {robot.id}] RL KICK mode")
        return target, kick, angle, False

    # ── emergency kick — ball is right there ────────────────────────────────
    if dist_to_ball < JUST_KICK_DIST and _approach_ok(robot.pos, ball.pos, goal):
        state["mode"] = "CHASE"
        state["dribbling"] = False
        print(f"[Attacker {robot.id}] EMERGENCY KICK")
        return goal.copy(), True, _angle_to(ball.pos, goal), False

    if dist_to_ball > 3.0:
        state["mode"] = "CHASE"
        state["dribbling"] = False

    # ── CHASE — approach ball, engage dribbler when close ───────────────────
    if state["mode"] == "CHASE":
        if dist_to_ball < 0.6:
            state["mode"] = "DECIDE"

        target = ball.pos.copy()
        # separate from teammates only when far
        if dist_to_ball > 0.5:
            for tm in teammates:
                diff = target - tm.pos
                d = np.linalg.norm(diff)
                if d < MIN_ROBOT_SEPARATION and d > 1e-6:
                    target += (diff / d) * (MIN_ROBOT_SEPARATION - d)

        # engage dribbler when close enough to scoop the ball
        dribble = dist_to_ball < DRIBBLE_ENGAGE_DIST
        if dribble:
            state["dribbling"] = True

        print(f"[Attacker {robot.id}] CHASE dribble={dribble}")
        return target, False, None, dribble

    # ── DECIDE — with ball, decide what to do ───────────────────────────────
    if state["mode"] == "DECIDE":
        if dist_to_ball > 1.0:
            state["mode"] = "CHASE"
            state["dribbling"] = False
            return ball.pos.copy(), False, None, False

        shot_target = best_shot_target(robot, opponents, goal)

        if shot_target is not None:
            state["mode"] = "SHOOT"
            state["shot_target"] = shot_target
            print(f"[Attacker {robot.id}] DECIDE → SHOOT (clear angle)")
        else:
            # no clear shot — check if dribbling can reposition us
            # carry ball toward a better angle
            state["mode"] = "CARRY"
            state["carry_start"] = robot.pos.copy()
            print(f"[Attacker {robot.id}] DECIDE → CARRY (reposition with ball)")

        return ball.pos.copy(), False, None, state["dribbling"]

    # ── CARRY — dribble the ball sideways to get a better angle ─────────────
    if state["mode"] == "CARRY":
        if dist_to_ball > 1.0:
            state["mode"] = "CHASE"
            state["dribbling"] = False
            return ball.pos.copy(), False, None, False

        carry_start = state.get("carry_start")
        if carry_start is not None:
            carried = float(np.linalg.norm(robot.pos - carry_start))
        else:
            carried = 0.0

        # check if we have a clear shot now
        shot_target = best_shot_target(robot, opponents, goal)
        if shot_target is not None or carried > DRIBBLE_CARRY_DIST:
            state["mode"] = "SHOOT"
            state["shot_target"] = shot_target if shot_target is not None else goal.copy()
            print(f"[Attacker {robot.id}] CARRY → SHOOT (carried={carried:.2f}m)")
            return ball.pos.copy(), False, None, True

        # dribble sideways — find best side to open up angle
        to_goal = goal - ball.pos
        perp = np.array([-to_goal[1], to_goal[0]])
        perp = perp / (np.linalg.norm(perp) + 1e-6)
        carry_target = ball.pos + perp * 0.8

        print(f"[Attacker {robot.id}] CARRY dribbling sideways")
        return carry_target, False, _angle_to(robot.pos, goal), True

    # ── SHOOT — line up behind ball toward open angle ────────────────────────
    if state["mode"] == "SHOOT":
        if dist_to_ball > 1.5:
            state["mode"] = "CHASE"
            state["dribbling"] = False
            return ball.pos.copy(), False, None, False

        shot_target = state.get("shot_target") if state.get("shot_target") is not None else goal
        behind = _behind_ball(ball.pos, shot_target)
        dist_to_behind = float(np.linalg.norm(robot.pos - behind))
        aligned = _approach_ok(robot.pos, ball.pos, shot_target)

        if dist_to_behind < BEHIND_THRESH and aligned:
            state["mode"] = "KICK"
            state["dribbling"] = False  # release dribbler before kick
            print(f"[Attacker {robot.id}] LINED UP → KICK")

        print(f"[Attacker {robot.id}] SHOOT behind={dist_to_behind:.2f} aligned={aligned}")
        return behind, False, None, False  # no dribble during lineup

    # ── KICK — drive through ball ────────────────────────────────────────────
    if state["mode"] == "KICK":
        if dist_to_ball > 1.0:
            state["mode"] = "CHASE"
            return ball.pos.copy(), False, None, False

        kick_angle = _angle_to(ball.pos, goal)

        if dist_to_ball < KICK_DIST:
            state["mode"] = "CHASE"
            print(f"[Attacker {robot.id}] KICK")
            return goal.copy(), True, kick_angle, False

        to_goal = goal - ball.pos
        dist_to_goal = np.linalg.norm(to_goal)
        drive_through = ball.pos + (to_goal / (dist_to_goal + 1e-6)) * 0.5
        print(f"[Attacker {robot.id}] DRIVE THROUGH")
        return drive_through, False, kick_angle, False

    # ── PASS ─────────────────────────────────────────────────────────────────
    if state["mode"] == "PASS":
        if dist_to_ball > 1.5:
            state["mode"] = "CHASE"
            return ball.pos.copy(), False, None, False

        if dist_to_ball < KICK_DIST:
            pass_target = state.get("pass_target") if state.get("pass_target") is not None else ball.pos
            state["mode"] = "CHASE"
            state["dribbling"] = False
            print(f"[Attacker {robot.id}] PASS")
            return pass_target.copy(), True, None, False

        pass_target = state.get("pass_target") if state.get("pass_target") is not None else ball.pos
        behind = _behind_ball(ball.pos, pass_target)
        return behind, False, None, False

    state["mode"] = "CHASE"
    return ball.pos.copy(), False, None, False