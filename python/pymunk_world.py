"""Headless pymunk world shared by simulation_node.py and the RL env.

ZMQ transport, game clock, and ball-stuck detection live in simulation_node.py;
this file is just physics.
"""

from __future__ import annotations

import math
import os
import random
import sys
from collections.abc import Iterable
from typing import Any
from decision_making.skills.dribble import try_dribble_ball

import numpy as np
import pymunk

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))  # project root (for decision_making.*)
sys.path.insert(0, _HERE)
from config import (  # noqa: E402
    ANGULAR_DAMP,
    BALL_DAMP,
    BALL_MASS,
    BALL_RADIUS,
    DT,
    FIELD_H,
    FIELD_W,
    GOAL_Y_MAX,
    GOAL_Y_MIN,
    LINEAR_DAMP,
    MOTOR_MAX_FORCE,
    NUM_ROBOTS,
    ROBOT_MASS,
    ROBOT_RADIUS,
    WHEEL_ANGLES,
    WHEEL_DISTANCE,
)

from decision_making.skills.kick import try_kick_ball  # noqa: E402

COLLISION_ROBOT = 1
COLLISION_BALL = 2
COLLISION_WALL = 3

# Wall half-thickness, meters. Must exceed |v_max| * DT so pymunk's discrete
# collision detection catches fast balls without CCD.
WALL_HALF_THICKNESS = 0.10

SPAWN_JITTER_STD = 0.10 # meters per axis, only applied when a seed is given

# Demo spawn layout: 2 blue offensive players (ids 0-1) versus a single
# red goalie (id 2) parked in front of the right-side goal.
NOMINAL_SPAWNS: list[tuple[float, float, float]] = [
    (3.0, 1.8, 0.0),
    (3.0, 4.2, 0.0),
    (8.5, 3.0, math.pi),
]


def _make_robot(
    space: pymunk.Space, x: float, y: float, angle: float = 0.0
) -> pymunk.Body:
    moment = pymunk.moment_for_circle(ROBOT_MASS, 0, ROBOT_RADIUS)
    body = pymunk.Body(ROBOT_MASS, moment)
    body.position = (x, y)
    body.angle = angle
    shape = pymunk.Circle(body, ROBOT_RADIUS)
    shape.elasticity = 0.3
    shape.friction = 0.5
    space.add(body, shape)
    return body


def _make_ball(space: pymunk.Space, x: float, y: float) -> pymunk.Body:
    moment = pymunk.moment_for_circle(BALL_MASS, 0, BALL_RADIUS)
    body = pymunk.Body(BALL_MASS, moment)
    body.position = (x, y)
    shape = pymunk.Circle(body, BALL_RADIUS)
    shape.elasticity = 0.6
    shape.friction = 0.4
    shape.collision_type = COLLISION_BALL
    space.add(body, shape)
    return body


def _add_walls(space: pymunk.Space) -> None:
    r = WALL_HALF_THICKNESS
    segments = [
        ((r, -r), (FIELD_W - r, -r)),
        ((r, FIELD_H + r), (FIELD_W - r, FIELD_H + r)),
        ((-r, r), (-r, GOAL_Y_MIN - r)),
        ((-r, GOAL_Y_MAX + r), (-r, FIELD_H - r)),
        ((FIELD_W + r, r), (FIELD_W + r, GOAL_Y_MIN - r)),
        ((FIELD_W + r, GOAL_Y_MAX + r), (FIELD_W + r, FIELD_H - r)),
    ]
    for a, b in segments:
        seg = pymunk.Segment(space.static_body, a, b, r)
        seg.elasticity = 0.8
        seg.friction = 0.5
        seg.collision_type = COLLISION_WALL
        space.add(seg)


def _apply_damping(body: pymunk.Body, lin_damp: float, ang_damp: float, dt: float) -> None:
    lin_factor = max(0.0, 1.0 - lin_damp * dt)
    ang_factor = max(0.0, 1.0 - ang_damp * dt)
    body.velocity = body.velocity * lin_factor
    body.angular_velocity *= ang_factor


def _apply_wheel_commands(body: pymunk.Body, wheel_speeds: list[float]) -> None:
    total_force = np.zeros(2)
    total_torque = 0.0
    for i, alpha in enumerate(WHEEL_ANGLES):
        force_mag = wheel_speeds[i] * MOTOR_MAX_FORCE
        drive_angle = alpha + math.pi / 2.0
        total_force += (
            np.array([math.cos(drive_angle), math.sin(drive_angle)]) * force_mag
        )
        total_torque += force_mag * WHEEL_DISTANCE

    a = body.angle
    wx = math.cos(a) * total_force[0] - math.sin(a) * total_force[1]
    wy = math.sin(a) * total_force[0] + math.cos(a) * total_force[1]
    body.apply_force_at_world_point((wx, wy), body.position)
    body.torque += total_torque


def _scoring_team(ball: pymunk.Body) -> str | None:
    y = ball.position.y
    if y < GOAL_Y_MIN or y > GOAL_Y_MAX:
        return None
    if ball.position.x <= 0.0:
        return "blue"
    if ball.position.x >= FIELD_W:
        return "red"
    return None


def _jittered(
    rng: random.Random | None, x: float, y: float
) -> tuple[float, float]:
    if rng is None:
        return x, y
    return (
        x + rng.gauss(0.0, SPAWN_JITTER_STD),
        y + rng.gauss(0.0, SPAWN_JITTER_STD),
    )


class PymunkWorld:
    def __init__(self, seed: int | None = None):
        self._rng: random.Random | None = None
        if seed is not None:
            random.seed(seed)
            np.random.seed(seed)
            self._rng = random.Random(seed)

        self.space: pymunk.Space = pymunk.Space()
        self.space.gravity = (0, 0)
        _add_walls(self.space)

        self.robots: list[pymunk.Body] = []
        for x, y, angle in NOMINAL_SPAWNS[:NUM_ROBOTS]:
            jx, jy = _jittered(self._rng, x, y)
            self.robots.append(_make_robot(self.space, jx, jy, angle))

        self.ball: pymunk.Body = _make_ball(self.space, FIELD_W / 2, FIELD_H / 2)
        self.sim_time: float = 0.0

    def step(
        self,
        wheel_cmds: dict[int, list[float]] | None = None,
        kicks: Iterable[int] | None = None,
        dribbling: set[int] | None = None,
    ) -> dict[str, Any]:
        # wheel_cmds: rid -> 3 wheel speeds in [-1, 1]. Missing robots get zeros.
        # kicks: rids to attempt a kick this tick. Returned "kicks" is rids actually fired.
        wheel_cmds = wheel_cmds or {}
        kicks = kicks or ()

        for i, body in enumerate(self.robots):
            speeds = wheel_cmds.get(i, [0.0, 0.0, 0.0])
            _apply_wheel_commands(body, speeds)
            _apply_damping(body, LINEAR_DAMP, ANGULAR_DAMP, DT)

        dribbling = dribbling or set()
        fired: list[int] = []
        for rid in kicks:
            if rid in dribbling:
                continue  # dribbling takes priority — skip kick
            if 0 <= rid < len(self.robots):
                if try_kick_ball(self.robots[rid], self.ball):
                    fired.append(rid)

        # apply dribbler forces
        for rid in dribbling:
            if 0 <= rid < len(self.robots):
                try_dribble_ball(self.robots[rid], self.ball, DT)

        _apply_damping(self.ball, BALL_DAMP, BALL_DAMP, DT)
        self.space.step(DT)
        self.sim_time += DT

        return {
            "t": self.sim_time,
            "kicks": fired,
            "scoring_team": _scoring_team(self.ball),
            "ball": {
                "x": self.ball.position.x,
                "y": self.ball.position.y,
                "vx": self.ball.velocity.x,
                "vy": self.ball.velocity.y,
            },
            "robots": {
                str(i): {
                    "x": body.position.x,
                    "y": body.position.y,
                    "angle": body.angle,
                    "vx": body.velocity.x,
                    "vy": body.velocity.y,
                    "omega": body.angular_velocity,
                }
                for i, body in enumerate(self.robots)
            },
        }

    def reset_ball(self, pos: tuple[float, float] | None = None) -> None:
        self.ball.velocity = (0.0, 0.0)
        self.ball.angular_velocity = 0.0
        if pos is None:
            self.ball.position = (FIELD_W / 2.0, FIELD_H / 2.0)
        else:
            self.ball.position = (float(pos[0]), float(pos[1]))

    def apply_dribble(self, robot_id: int, dt: float) -> None:
        if 0 <= robot_id < len(self.robots):
         try_dribble_ball(self.robots[robot_id], self.ball, dt)

    def set_robot(
        self,
        rid: int,
        pos: tuple[float, float],
        angle: float = 0.0,
    ) -> None:
        body = self.robots[rid]
        body.position = (float(pos[0]), float(pos[1]))
        body.angle = angle
        body.velocity = (0.0, 0.0)
        body.angular_velocity = 0.0
