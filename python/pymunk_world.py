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
CATEGORY_ROBOT = 0b001
CATEGORY_BALL  = 0b010
CATEGORY_WALL  = 0b100

# Wall half-thickness, meters. Must exceed |v_max| * DT so pymunk's discrete
# collision detection catches fast balls without CCD.
WALL_HALF_THICKNESS = 0.10

# Real SSL kick-speed cap. Doubles as a tunneling safety: ball travel/substep
# stays well under the wall band even at full speed.
MAX_BALL_SPEED = 6.5

# Physics substeps per outer tick. More iterations on the constraint solver
# resolve robot+ball+wall pin contacts that otherwise pop the ball through.
PHYSICS_SUBSTEPS = 4
SOLVER_ITERATIONS = 30

SPAWN_JITTER_STD = 0.10 # meters per axis, only applied when a seed is given

# 3v3 spawn layout, mirrored across x = FIELD_W/2: each team has a goalie
# (ids 0, 3) in front of its goal and two forwards on the half-field line.
NOMINAL_SPAWNS: list[tuple[float, float, float]] = [
    (0.5, 3.0, 0.0),
    (3.0, 1.5, 0.0),
    (3.0, 4.5, 0.0),
    (8.5, 3.0, math.pi),
    (6.0, 1.5, math.pi),
    (6.0, 4.5, math.pi),
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
    shape.filter = pymunk.ShapeFilter(
    categories=CATEGORY_ROBOT,
    mask=CATEGORY_ROBOT | CATEGORY_BALL | CATEGORY_WALL,
    )
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
    shape.filter = pymunk.ShapeFilter(
    categories=CATEGORY_BALL,
    mask=CATEGORY_ROBOT | CATEGORY_WALL,
    )
    space.add(body, shape)
    return body

def _add_walls(space: pymunk.Space) -> None:
    r = WALL_HALF_THICKNESS

    wall_filter = pymunk.ShapeFilter(
        categories=CATEGORY_WALL,
        mask=CATEGORY_ROBOT | CATEGORY_BALL,
    )

    robot_only_filter = pymunk.ShapeFilter(
        categories=CATEGORY_WALL,
        mask=CATEGORY_ROBOT,  # ball can pass
    )

    def add_seg(a, b, filt, elasticity=0.8):
        seg = pymunk.Segment(space.static_body, a, b, r)
        seg.elasticity = elasticity
        seg.friction = 0.5
        seg.collision_type = COLLISION_WALL
        seg.filter = filt
        space.add(seg)

    # --- TOP & BOTTOM WALLS (block everything)
    add_seg((r, -r), (FIELD_W - r, -r), wall_filter)                # bottom
    add_seg((r, FIELD_H + r), (FIELD_W - r, FIELD_H + r), wall_filter)  # top

    # --- LEFT WALL (split around goal gap)
    add_seg((-r, r), (-r, GOAL_Y_MIN), wall_filter)                 # below goal
    add_seg((-r, GOAL_Y_MAX), (-r, FIELD_H - r), wall_filter)       # above goal

    # --- RIGHT WALL (split around goal gap)
    add_seg((FIELD_W + r, r), (FIELD_W + r, GOAL_Y_MIN), wall_filter)
    add_seg((FIELD_W + r, GOAL_Y_MAX), (FIELD_W + r, FIELD_H - r), wall_filter)

    # --- BACK OF GOALS (robots blocked, ball passes through)
    goal_back_x_left = -ROBOT_RADIUS
    goal_back_x_right = FIELD_W + ROBOT_RADIUS

    add_seg(
        (goal_back_x_left, GOAL_Y_MIN),
        (goal_back_x_left, GOAL_Y_MAX),
        robot_only_filter,
        elasticity=0.3,
    )

    add_seg(
        (goal_back_x_right, GOAL_Y_MIN),
        (goal_back_x_right, GOAL_Y_MAX),
        robot_only_filter,
        elasticity=0.3,
    )

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
        self.space.iterations = SOLVER_ITERATIONS
        _add_walls(self.space)

        self.robots: list[pymunk.Body] = []
        for x, y, angle in NOMINAL_SPAWNS[:NUM_ROBOTS]:
            jx, jy = _jittered(self._rng, x, y)
            self.robots.append(_make_robot(self.space, jx, jy, angle))

        self.ball: pymunk.Body = _make_ball(self.space, FIELD_W / 2, FIELD_H / 2)
        self.sim_time: float = 0.0
        # Robot ids owned by the camera (vision_bridge). Kinematic: sim can't push them.
        self.mirrored: set[int] = set()

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
        dribbling = dribbling or set()

        # Kicks fire once per tick: impulses are instantaneous, not cleared
        # by space.step, so they don't need re-applying inside the substep loop.
        fired: list[int] = []
        for rid in kicks:
            if rid in self.mirrored:
                continue  # real robot kicks for itself
            if rid in dribbling:
                continue  # dribbling takes priority — skip kick
            if 0 <= rid < len(self.robots):
                if try_kick_ball(self.robots[rid], self.ball):
                    fired.append(rid)

        # Clamp before stepping so kicks + dribble pushes can't tunnel walls.
        bv = self.ball.velocity
        bs = bv.length
        if bs > MAX_BALL_SPEED:
            self.ball.velocity = bv * (MAX_BALL_SPEED / bs)

        # Forces (wheels, dribble spring, damping) must be re-applied inside
        # the substep loop: pymunk clears body.force/torque after each step().
        sub_dt = DT / PHYSICS_SUBSTEPS
        for _ in range(PHYSICS_SUBSTEPS):
            for i, body in enumerate(self.robots):
                if i in self.mirrored:
                    continue
                speeds = wheel_cmds.get(i, [0.0, 0.0, 0.0])
                _apply_wheel_commands(body, speeds)
                _apply_damping(body, LINEAR_DAMP, ANGULAR_DAMP, sub_dt)

            for rid in dribbling:
                if 0 <= rid < len(self.robots) and rid not in self.mirrored:
                    try_dribble_ball(self.robots[rid], self.ball, sub_dt)

            _apply_damping(self.ball, BALL_DAMP, BALL_DAMP, sub_dt)
            self.space.step(sub_dt)
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

    # ── vision mirror ────────────────────────────────────────────────────────

    def set_mirrored(self, rids: Iterable[int]) -> None:
        # Only touch bodies that change hands; already-mirrored ones keep tracking.
        new = {rid for rid in rids if 0 <= rid < len(self.robots)}
        for rid in self.mirrored - new:
            self.robots[rid].body_type = pymunk.Body.DYNAMIC
        for rid in new - self.mirrored:
            body = self.robots[rid]
            body.body_type = pymunk.Body.KINEMATIC
            body.velocity = (0.0, 0.0)
            body.angular_velocity = 0.0
        self.mirrored = new

    def drive_robot(
        self,
        rid: int,
        pos: tuple[float, float],
        vel: tuple[float, float],
        angle: float,
        omega: float,
        tau: float,
        snap_dist: float,
        snap_angle: float,
    ) -> None:
        # Velocity servo onto the measurement so contacts with sim bodies stay
        # physical; teleport only when too far off (reacquire, first sighting).
        body = self.robots[rid]
        ex, ey = pos[0] - body.position.x, pos[1] - body.position.y
        ea = (angle - body.angle + math.pi) % (2.0 * math.pi) - math.pi
        if math.hypot(ex, ey) > snap_dist or abs(ea) > snap_angle:
            body.position = (float(pos[0]), float(pos[1]))
            body.angle = angle
            body.velocity = (float(vel[0]), float(vel[1]))
            body.angular_velocity = omega
            self.space.reindex_shapes_for_body(body)
            return
        body.velocity = (vel[0] + ex / tau, vel[1] + ey / tau)
        body.angular_velocity = omega + ea / tau

    def coast_robot(self, rid: int) -> None:
        body = self.robots[rid]
        body.velocity = body.velocity * max(0.0, 1.0 - LINEAR_DAMP * DT)
        body.angular_velocity *= max(0.0, 1.0 - ANGULAR_DAMP * DT)

    def freeze_robot(self, rid: int) -> None:
        body = self.robots[rid]
        body.velocity = (0.0, 0.0)
        body.angular_velocity = 0.0

    def drive_ball(
        self,
        pos: tuple[float, float],
        vel: tuple[float, float],
        tau: float,
        snap_dist: float,
    ) -> None:
        # Ball stays dynamic so sim robots still collide with it; the servo
        # overwrites whatever they did on the next camera-driven tick.
        ex, ey = pos[0] - self.ball.position.x, pos[1] - self.ball.position.y
        if math.hypot(ex, ey) > snap_dist:
            self.ball.position = (float(pos[0]), float(pos[1]))
            self.ball.velocity = (float(vel[0]), float(vel[1]))
            self.space.reindex_shapes_for_body(self.ball)
            return
        self.ball.velocity = (vel[0] + ex / tau, vel[1] + ey / tau)
