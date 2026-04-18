"""
Adapter so rl_env can drive the red team with decision_making/strategy_node.

The RL policy controls blue; red needs to be something more adversarial
than a stationary traffic cone or blue drifts into a "kick once, then
orbit" local optimum (no-opponent = nothing punishes disengagement).

This module wraps strategy_node.decide — the same scripted team shipped
in the live ZMQ pipeline — so the policy trains against the opponent it
will actually face at deploy time. Heading is synthesized from the
target direction since strategy_node only publishes (x, y); the
low-level controller handles the rest.
"""
from __future__ import annotations

import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_DECISION_DIR = os.path.join(os.path.dirname(_HERE), "decision_making")
if _DECISION_DIR not in sys.path:
    sys.path.insert(0, _DECISION_DIR)

import strategy_node as _strategy  # noqa: E402
from state import build_game_state  # noqa: E402


def scripted_red_targets(
    raw_state: dict, num_active: int = 3,
) -> dict[int, tuple[float, float, float]]:
    """Return {robot_id: (target_x, target_y, theta)} for the active red
    robots. At most `num_active` of them get targets — remaining red
    robots stay stationary. Roles are assigned by ball-distance rank
    inside strategy_node.decide: closest=attacker, 2nd=support,
    3rd=defender. Dropping the lowest-priority roles (defender first)
    is the curriculum knob — full strength = 3, no goalie = 2, solo
    chaser = 1.

    Heading faces the target when it's non-trivial distance away,
    otherwise points at the ball — that's what the blue strategy
    approximates implicitly too (robot_node's controller chooses a
    heading if target_theta is None).
    """
    if num_active <= 0:
        return {}

    prev = _strategy.OUR_COLOR
    _strategy.OUR_COLOR = "red"
    try:
        gs = build_game_state(raw_state, our_color="red")
        raw_targets = _strategy.decide(gs)  # {rid: np.ndarray([x,y])}
    finally:
        _strategy.OUR_COLOR = prev

    # Keep only the top-`num_active` roles. `decide` assigns roles in
    # ball-distance rank order (attacker, supporter, defender) while
    # iterating the sorted list, so we filter to the closest robots
    # to the ball — that's the role ordering we want to keep.
    red_by_ball_distance = sorted(
        gs.red, key=lambda r: float(np.linalg.norm(gs.ball.pos - r.pos))
    )
    keep_ids = {r.id for r in red_by_ball_distance[:num_active]}
    raw_targets = {rid: pos for rid, pos in raw_targets.items() if rid in keep_ids}

    bx = float(raw_state["ball"]["x"])
    by = float(raw_state["ball"]["y"])

    out: dict[int, tuple[float, float, float]] = {}
    for rid, pos in raw_targets.items():
        r = raw_state["robots"][str(rid)]
        dx = float(pos[0]) - float(r["x"])
        dy = float(pos[1]) - float(r["y"])
        if dx * dx + dy * dy > 1e-4:
            theta = float(np.arctan2(dy, dx))
        else:
            theta = float(np.arctan2(by - float(r["y"]), bx - float(r["x"])))
        out[int(rid)] = (float(pos[0]), float(pos[1]), theta)
    return out
