import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from geometry import opp_goal
from state import GameState, RobotState
from BT.conditions import best_shot_target, teammate_open

KICK_DIST = 0.15
BEHIND_DIST = 0.5
BEHIND_THRESH = 0.2

_state = {"mode": "CHASE"}

def _behind_ball(ball_pos, shoot_target):
    """Get position behind ball relative to shoot target."""
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

    if dist_to_ball > 3.0:
        _state["mode"] = "CHASE"

    # CHASE — get close to ball
    if _state["mode"] == "CHASE":
        if dist_to_ball < 0.6:
            _state["mode"] = "DECIDE"
        print(f"[Attacker {robot.id}] CHASE")
        return ball.pos.copy(), False

    # DECIDE — conditions.py figures out what to do
    if _state["mode"] == "DECIDE":
        if dist_to_ball > 1.0:
            _state["mode"] = "CHASE"
            return ball.pos.copy(), False

        # ask conditions.py for the best open angle on goal
        shot_target = best_shot_target(robot, opponents, goal)

        if shot_target is not None:
            # there IS an open angle — store it and line up
            _state["mode"] = "SHOOT"
            _state["shot_target"] = shot_target
            print(f"[Attacker {robot.id}] → SHOOT (open angle found)")
        else:
            # no open shot — check if we can pass
            open_teammate = next(
                (tm for tm in teammates if teammate_open(robot, tm, opponents)),
                None
            )
            if open_teammate is not None:
                _state["mode"] = "PASS"
                _state["pass_target"] = open_teammate.pos.copy()
                print(f"[Attacker {robot.id}] → PASS to {open_teammate.id}")
            else:
                # nothing open — reposition around ball
                _state["mode"] = "REPOSITION"
                print(f"[Attacker {robot.id}] → REPOSITION")

        return ball.pos.copy(), False

    # SHOOT — line up behind ball toward open angle, then kick
    if _state["mode"] == "SHOOT":
        if dist_to_ball > 1.5:
            _state["mode"] = "CHASE"
            return ball.pos.copy(), False

        shot_target = _state.get("shot_target", goal)
        behind = _behind_ball(ball.pos, shot_target)
        dist_to_behind = float(np.linalg.norm(robot.pos - behind))

        if dist_to_behind < BEHIND_THRESH:
            _state["mode"] = "KICK"

        print(f"[Attacker {robot.id}] LINE UP")
        return behind, False

    # KICK — drive through ball
    if _state["mode"] == "KICK":
        if dist_to_ball > 1.0:
            _state["mode"] = "CHASE"
            return ball.pos.copy(), False

        if dist_to_ball < KICK_DIST:
            shot_target = _state.get("shot_target", goal)
            _state["mode"] = "CHASE"
            print(f"[Attacker {robot.id}] SHOOT")
            return shot_target.copy(), True

        print(f"[Attacker {robot.id}] DRIVE")
        return ball.pos.copy(), False

    # PASS — drive into ball and kick toward teammate
    if _state["mode"] == "PASS":
        if dist_to_ball > 1.5:
            _state["mode"] = "CHASE"
            return ball.pos.copy(), False

        if dist_to_ball < KICK_DIST:
            pass_target = _state.get("pass_target", ball.pos)
            _state["mode"] = "CHASE"
            print(f"[Attacker {robot.id}] PASS")
            return pass_target.copy(), True

        # line up behind ball toward teammate
        pass_target = _state.get("pass_target", ball.pos)
        behind = _behind_ball(ball.pos, pass_target)
        return behind, False

    # REPOSITION — orbit to find open angle
    if _state["mode"] == "REPOSITION":
        if dist_to_ball > 2.0:
            _state["mode"] = "CHASE"
            return ball.pos.copy(), False

        best_pos = None
        best_clear = 0.0
        for i in range(8):
            angle = (i / 8) * 2 * np.pi
            candidate = ball.pos + np.array([np.cos(angle), np.sin(angle)]) * BEHIND_DIST
            from BT.conditions import best_shot_target as bst
            class FakeRobot:
                pos = candidate
            shot = bst(FakeRobot(), opponents, goal)
            if shot is not None:
                dist = float(np.linalg.norm(robot.pos - candidate))
                score = 1.0 / (dist + 0.1)
                if score > best_clear:
                    best_clear = score
                    best_pos = candidate

        if best_pos is not None:
            dist_to_best = float(np.linalg.norm(robot.pos - best_pos))
            if dist_to_best < BEHIND_THRESH:
                _state["mode"] = "DECIDE"
            print(f"[Attacker {robot.id}] REPOSITION")
            return best_pos, False

        _state["mode"] = "DECIDE"
        return ball.pos.copy(), False

    _state["mode"] = "CHASE"
    return ball.pos.copy(), False