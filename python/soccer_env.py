"""
In-process soccer environment built directly on pymunk.

SoccerEnv is an alternative entry point into the same physics world
that simulation_node.py publishes over ZMQ. It exposes a classic
`reset(seed) / step(action) / get_obs()` API and runs entirely in the
caller's process — no ZMQ, no subprocess, no wall-clock sleep.

This is the foundation for future RL training code. It deliberately
does NOT:

  - assume any particular RL framework (gym, gymnasium, sb3, ...),
  - impose a reward function (step returns 0.0),
  - terminate episodes (step returns done=False),
  - add observation noise or latency,
  - translate high-level actions into wheel commands.

All of those are separate concerns that will live in wrappers on top
of this class so the physics layer stays small and testable.

The physics helpers themselves (`_make_robot`, `_add_walls`, kick
impulse, damping, scoring, ...) are imported from simulation_node so
the RL path and the ZMQ path share one pymunk model — fixing a bug in
one automatically fixes it in the other.
"""

from __future__ import annotations

import math
import os
import random
import sys
from typing import Any

import pymunk

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from config import BALL_DAMP, DT, FIELD_H, FIELD_W, NUM_ROBOTS
from simulation_node import (
    _add_walls,
    _apply_damping,
    _apply_damping_custom,
    _apply_wheel_commands,
    _jittered,
    _make_ball,
    _make_robot,
    _reset_ball_to_center,
    _scoring_team,
    _try_kick_ball,
)

#: Initial (x, y, angle) for every robot in ID order. Matches the
#: nominal spawn formation in simulation_node.main() so runs under
#: this env line up with ZMQ-mode runs when the seed is the same.
NOMINAL_SPAWNS: list[tuple[float, float, float]] = [
    (2.0, 3.0, 0.0),      # 0: blue attacker
    (1.5, 4.5, 0.0),      # 1: blue supporter
    (0.8, 3.0, 0.0),      # 2: blue defender
    (7.0, 3.0, math.pi),  # 3: red attacker
    (7.5, 1.5, math.pi),  # 4: red supporter
    (8.2, 3.0, math.pi),  # 5: red defender
]


class SoccerEnv:
    """
    Minimal, framework-agnostic soccer environment.

    Action schema (every field optional)::

        {
            "wheels": {robot_id: [w0, w1, w2], ...},  # missing robots → zeros
            "kicks":  [robot_id, ...],                 # robots firing this tick
        }

    robot_id may be int or str; missing entries default to "no command".

    Observation: a dict with the same shape simulation_node publishes on
    VISION_PORT, minus ``last_goal``. Use ``info["goal"]`` to detect
    scoring events.

    Reward / done: stub. Always returns ``(reward=0.0, done=False)``.
    Wrappers are responsible for reward shaping and episode termination.
    """

    def __init__(self) -> None:
        self.space: pymunk.Space | None = None
        self.robots: list[pymunk.Body] = []
        self.ball: pymunk.Body | None = None
        self.score: dict[str, int] = {"blue": 0, "red": 0}
        self.sim_time: float = 0.0
        self._rng: random.Random | None = None
        self._seed: int | None = None

    # ── lifecycle ────────────────────────────────────────────────────────────

    def reset(self, seed: int | None = None) -> dict:
        """
        Build a fresh physics world and return the initial observation.

        If ``seed`` is given, robot spawn positions are jittered by the
        same small Gaussian that simulation_node's --seed flag uses.
        Passing the same seed twice yields byte-identical initial state.
        """
        if seed is not None:
            self._seed = seed
            self._rng = random.Random(seed)
        elif self._rng is None:
            # No seed ever provided → deterministic nominal spawns.
            self._rng = None

        self.space = pymunk.Space()
        self.space.gravity = (0, 0)
        _add_walls(self.space)

        self.robots = []
        for x, y, angle in NOMINAL_SPAWNS:
            jx, jy = _jittered(self._rng, x, y)
            self.robots.append(_make_robot(self.space, jx, jy, angle))

        self.ball = _make_ball(self.space, FIELD_W / 2.0, FIELD_H / 2.0)
        self.score = {"blue": 0, "red": 0}
        self.sim_time = 0.0

        return self.get_obs()

    def step(
        self, action: dict | None = None
    ) -> tuple[dict, float, bool, dict[str, Any]]:
        """
        Advance one physics tick. Returns ``(obs, reward, done, info)``.

        ``info`` always contains:
          - ``goal``:  "blue" | "red" | None — team that scored on this step
          - ``kicked``: list of robot ids whose kick actually connected
        """
        if self.space is None or self.ball is None:
            raise RuntimeError("SoccerEnv.step() called before reset()")

        action = action or {}
        wheels = action.get("wheels", {}) or {}
        kicks = action.get("kicks", []) or []

        # Apply wheel commands (robots without an entry get zero torque).
        for i, body in enumerate(self.robots):
            ws = wheels.get(i)
            if ws is None:
                ws = wheels.get(str(i), [0.0, 0.0, 0.0])
            _apply_wheel_commands(body, ws)
            _apply_damping(body, DT)

        # Apply kicks BEFORE stepping physics so the impulse is integrated
        # together with any velocity already on the ball.
        kicked: list[int] = []
        for rid in kicks:
            rid_i = int(rid)
            if 0 <= rid_i < NUM_ROBOTS:
                if _try_kick_ball(self.robots[rid_i], self.ball):
                    kicked.append(rid_i)

        _apply_damping_custom(self.ball, BALL_DAMP, BALL_DAMP, DT)
        self.space.step(DT)
        self.sim_time += DT

        scoring_team = _scoring_team(self.ball)
        if scoring_team is not None:
            self.score[scoring_team] += 1
            _reset_ball_to_center(self.ball)

        info: dict[str, Any] = {"goal": scoring_team, "kicked": kicked}
        return self.get_obs(), 0.0, False, info

    # ── observation ──────────────────────────────────────────────────────────

    def get_obs(self) -> dict:
        """
        Current world state, same shape as simulation_node's VISION_PORT
        payload (minus ``last_goal``).
        """
        if self.ball is None:
            raise RuntimeError("SoccerEnv.get_obs() called before reset()")
        return {
            "t": self.sim_time,
            "score": dict(self.score),
            "ball": {
                "x": self.ball.position.x,
                "y": self.ball.position.y,
                "vx": self.ball.velocity.x,
                "vy": self.ball.velocity.y,
            },
            "robots": {
                str(i): {
                    "x": b.position.x,
                    "y": b.position.y,
                    "angle": b.angle,
                    "vx": b.velocity.x,
                    "vy": b.velocity.y,
                    "omega": b.angular_velocity,
                }
                for i, b in enumerate(self.robots)
            },
        }
