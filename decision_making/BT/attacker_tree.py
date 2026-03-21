import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from geometry import opp_goal
from state import GameState, RobotState
from BT.conditions import best_shot_target, teammate_open, avoid_teammates, MIN_ROBOT_SEPARATION

KICK_DIST = 0.15
BEHIND_DIST = 0.5
BEHIND_THRESH = 0.2
JUST_KICK_DIST = 0.3  # if ball is this close, skip all logic and just shoot

# per-robot state keyed by robot id to avoid shared state between teams
_states = {}

def _get_state(robot_id):
    if robot_id not in _states:
        _states[robot_id] = {"mode": "CHASE", "shot_target": None, "pass_target": None}
    return _states[robot_id]

def _behind_ball(ball_pos, shoot_target):
    to_target = shoot_target - ball_pos
    dist = np.linalg.norm(to_target)
    if dist < 1e-6:
        return ball_pos + np.array([-BEHIND_DIST, 0])
    return ball_pos - (to_target / dist) * BEHIND_DIST

def attacker_decide(robot: RobotState, gamestate: GameState, color: str) -> tuple[np.ndarray, bool]:
    ball = gamestate.ball
    goal = opp_goal(color)
    opponents = gamestate.opponents_team
    teammates = [r for r in gamestate.our_team if r.id != robot.id]
    dist_to_ball = float(np.linalg.norm(robot.pos - ball.pos))

    state = _get_state(robot.id)

    # ── if ball is right there, just kick regardless of everything ──────────
    if dist_to_ball < JUST_KICK_DIST:
        state["mode"] = "CHASE"
        return goal.copy(), True

    if dist_to_ball > 3.0:
        state["mode"] = "CHASE"

    # CHASE — get close to ball, only avoid teammates if not very close to ball
    if state["mode"] == "CHASE":
        if dist_to_ball < 0.6:
            state["mode"] = "DECIDE"
        target = ball.pos.copy()
        if dist_to_ball > 0.8:  # only nudge away from teammates when not close to ball
            for tm in teammates:
                diff = target - tm.pos
                dist = np.linalg.norm(diff)
                if dist < MIN_ROBOT_SEPARATION and dist > 1e-6:
                    target += (diff / dist) * (MIN_ROBOT_SEPARATION - dist)
        return target, False

    # DECIDE — pick action, but if nothing works just shoot anyway
    if state["mode"] == "DECIDE":
        if dist_to_ball > 1.0:
            state["mode"] = "CHASE"
            return ball.pos.copy(), False

        shot_target = state.get("shot_target") if state.get("shot_target") is not None else goal

        if shot_target is not None or dist_to_ball < 0.3:
            state["mode"] = "SHOOT"
            state["shot_target"] = shot_target if shot_target is not None else goal
        else:
            open_teammate = next(
                (tm for tm in teammates if teammate_open(robot, tm, opponents)),
                None
            )
            if open_teammate is not None:
                state["mode"] = "PASS"
                state["pass_target"] = open_teammate.pos.copy()
            else:
                # no clear shot or pass — just shoot anyway rather than reposition
                state["mode"] = "SHOOT"
                state["shot_target"] = goal.copy()

        return ball.pos.copy(), False

    # SHOOT — line up behind ball
    if state["mode"] == "SHOOT":
        if dist_to_ball > 1.5:
            state["mode"] = "CHASE"
            return ball.pos.copy(), False

        shot_target = state.get("shot_target") if state.get("shot_target") is not None else goal
        behind = _behind_ball(ball.pos, shot_target)
        dist_to_behind = float(np.linalg.norm(robot.pos - behind))

        # check alignment — robot→ball direction should point toward goal
        to_ball = ball.pos - robot.pos
        to_goal = shot_target - ball.pos
        dist_tb = np.linalg.norm(to_ball)
        dist_tg = np.linalg.norm(to_goal)

        if dist_tb > 1e-6 and dist_tg > 1e-6:
            alignment = float(np.dot(to_ball / dist_tb, to_goal / dist_tg))
        else:
            alignment = 0.0

        # only transition to KICK when both close enough AND aligned (dot product > 0.85 ≈ within ~30°)
        if dist_to_behind < BEHIND_THRESH and alignment > 0.85:
            state["mode"] = "KICK"
        return behind, False

    # KICK — drive through ball and shoot
    if state["mode"] == "KICK":
        if dist_to_ball > 1.0:
            state["mode"] = "CHASE"
            return ball.pos.copy(), False

        if dist_to_ball < KICK_DIST:
            state["mode"] = "CHASE"
            return goal.copy(), True

        # target a point past the ball toward goal so robot drives through it
        to_goal = goal - ball.pos
        dist_to_goal = np.linalg.norm(to_goal)
        drive_through = ball.pos + (to_goal / (dist_to_goal + 1e-6)) * 0.4
        return drive_through, False

    # PASS — line up and kick toward teammate
    if state["mode"] == "PASS":
        if dist_to_ball > 1.5:
            state["mode"] = "CHASE"
            return ball.pos.copy(), False

        if dist_to_ball < KICK_DIST:
            pass_target = state.get("pass_target") if state.get("pass_target") is not None else ball.pos
            state["mode"] = "CHASE"
            return pass_target.copy(), True

        pass_target = state.get("pass_target") or ball.pos
        behind = _behind_ball(ball.pos, pass_target)
        return behind, False

    state["mode"] = "CHASE"
    return ball.pos.copy(), False