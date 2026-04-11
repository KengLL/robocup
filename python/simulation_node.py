#!/usr/bin/env python3
"""
Simulation Node — Pymunk physics engine (replaces real camera/world).

Publishes world state on VISION_PORT (ZMQ PUB).
Receives robot wheel-speed commands on COMMAND_PORT (ZMQ PULL).

In real-world deployment, swap this node for a Vision Node that reads
AprilTag data — the rest of the pipeline stays identical.

CLI flags:
  --headless   Do not sleep at the end of each tick. The physics loop
               advances as fast as the CPU allows. Useful for CI and
               (eventually) RL training.
  --seed INT   Seed Python's `random` module and numpy RNG for
               reproducibility. When set, initial robot positions are
               jittered by a small Gaussian so repeated runs at the
               same seed are identical but runs at different seeds
               explore slightly different starting states. When unset
               (default), spawn positions and behavior are exactly
               identical to pre-flag runs.
"""

import argparse
import json
import math
import os
import random
import sys
import time

import numpy as np
import pymunk
import zmq

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from config import *

# collision types
# used for later collision logic
COLLISION_ROBOT = 1
COLLISION_BALL = 2
COLLISION_WALL = 3

# Goal mouth geometry is defined in config.py (GOAL_MOUTH_H, GOAL_Y_MIN,
# GOAL_Y_MAX) so viz and sim share one source of truth.

# Kick model: small rectangular contact zone in front of robot.
KICK_ZONE_DEPTH = 0.12
KICK_ZONE_HALF_WIDTH = 0.10
KICK_IMPULSE = 0.22


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


#: Half-thickness of the field boundary walls (meters). Must be larger
#: than the maximum ball displacement per physics tick (|v_max| * DT)
#: so pymunk's discrete collision detection catches fast-moving balls
#: without needing CCD or a post-step teleport hack.
WALL_HALF_THICKNESS = 0.10


def _add_walls(space: pymunk.Space) -> None:
    """
    Build the field boundary as thick static segments. Each segment's
    center line sits `r` outside the nominal field edge and its endpoints
    are shortened by `r` along the segment direction, so the capsule
    collision shape's INNER surface lines up exactly with the field
    boundary and does not protrude into the goal mouth.
    """
    r = WALL_HALF_THICKNESS

    segments = [
        # y=0 touchline  (center at y=-r, inner edge at y=0)
        ((r, -r), (FIELD_W - r, -r)),
        # y=FIELD_H touchline  (center at y=FIELD_H+r, inner edge at y=FIELD_H)
        ((r, FIELD_H + r), (FIELD_W - r, FIELD_H + r)),
        # x=0 sideline, framing the goal mouth
        ((-r, r), (-r, GOAL_Y_MIN - r)),
        ((-r, GOAL_Y_MAX + r), (-r, FIELD_H - r)),
        # x=FIELD_W sideline, framing the goal mouth
        ((FIELD_W + r, r), (FIELD_W + r, GOAL_Y_MIN - r)),
        ((FIELD_W + r, GOAL_Y_MAX + r), (FIELD_W + r, FIELD_H - r)),
    ]

    for a, b in segments:
        seg = pymunk.Segment(space.static_body, a, b, r)
        seg.elasticity = 0.8
        seg.friction = 0.5
        seg.collision_type = COLLISION_WALL
        space.add(seg)


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


def _apply_damping(body: pymunk.Body, dt: float) -> None:
    """Manual per-step damping matching Godot RigidBody2D linear_damp."""
    lin_factor = max(0.0, 1.0 - LINEAR_DAMP * dt)
    ang_factor = max(0.0, 1.0 - ANGULAR_DAMP * dt)
    body.velocity = body.velocity * lin_factor
    body.angular_velocity *= ang_factor


def _apply_damping_custom(
    body: pymunk.Body, lin_damp: float, ang_damp: float, dt: float
) -> None:
    lin_factor = max(0.0, 1.0 - lin_damp * dt)
    ang_factor = max(0.0, 1.0 - ang_damp * dt)
    body.velocity = body.velocity * lin_factor
    body.angular_velocity *= ang_factor


def _apply_wheel_commands(body: pymunk.Body, wheel_speeds: list) -> None:
    """
    Forward kinematics: wheel speeds → forces on the pymunk body.
    Matches robot.gd _physics_process forward kinematics block.
    """
    total_force = np.zeros(2)
    total_torque = 0.0
    for i, alpha in enumerate(WHEEL_ANGLES):
        force_mag = wheel_speeds[i] * MOTOR_MAX_FORCE
        drive_angle = alpha + math.pi / 2.0
        total_force += (
            np.array([math.cos(drive_angle), math.sin(drive_angle)]) * force_mag
        )
        total_torque += force_mag * WHEEL_DISTANCE

    # Rotate local force vector to world frame (matches apply_central_force(rotated(rotation)))
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


def _reset_ball_to_center(ball: pymunk.Body) -> None:
    ball.velocity = (0.0, 0.0)
    ball.angular_velocity = 0.0
    ball.position = (FIELD_W / 2.0, FIELD_H / 2.0)


def _try_kick_ball(robot: pymunk.Body, ball: pymunk.Body) -> bool:
    """Kick if ball is in a small front tangent zone of the robot."""
    dx = ball.position.x - robot.position.x
    dy = ball.position.y - robot.position.y
    a = robot.angle

    # World -> robot local frame where +x is robot forward.
    local_x = math.cos(a) * dx + math.sin(a) * dy
    local_y = -math.sin(a) * dx + math.cos(a) * dy

    zone_min_x = ROBOT_RADIUS
    zone_max_x = ROBOT_RADIUS + KICK_ZONE_DEPTH
    in_zone = zone_min_x <= local_x <= zone_max_x and abs(local_y) <= KICK_ZONE_HALF_WIDTH
    if not in_zone:
        return False

    impulse = (math.cos(a) * KICK_IMPULSE, math.sin(a) * KICK_IMPULSE)
    ball.apply_impulse_at_world_point(impulse, ball.position)
    return True


#: Per-axis standard deviation (meters) of the Gaussian jitter applied
#: to initial robot positions when a seed is provided. Small enough that
#: the nominal formation still makes sense; large enough that policies
#: cannot memorize exact spawn locations.
SPAWN_JITTER_STD = 0.10


def _jittered(
    rng: random.Random | None, x: float, y: float
) -> tuple[float, float]:
    if rng is None:
        return x, y
    return (
        x + rng.gauss(0.0, SPAWN_JITTER_STD),
        y + rng.gauss(0.0, SPAWN_JITTER_STD),
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="RoboCup simulation node")
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run the physics loop as fast as the CPU allows (no tick sleep).",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Seed for random / numpy.random. Also enables Gaussian jitter "
             "of initial robot positions for domain randomization.",
    )
    args = parser.parse_args()

    rng: random.Random | None = None
    if args.seed is not None:
        random.seed(args.seed)
        np.random.seed(args.seed)
        rng = random.Random(args.seed)
        print(f"[SimNode] seed = {args.seed}  (spawn jitter enabled)")

    space = pymunk.Space()
    space.gravity = (0, 0)
    _add_walls(space)

    # blue team (robots 0-2) — left side
    # red team (robots 3-5) — right side
    nominal_spawns = [
        # (x, y, angle, label)
        (2.0, 3.0, 0.0,      "blue attacker"),
        (1.5, 4.5, 0.0,      "blue supporter"),
        (0.8, 3.0, 0.0,      "blue defender"),
        (7.0, 3.0, math.pi,  "red attacker"),
        (7.5, 1.5, math.pi,  "red supporter"),
        (8.2, 3.0, math.pi,  "red defender"),
    ]
    robots = []
    for x, y, angle, _label in nominal_spawns:
        jx, jy = _jittered(rng, x, y)
        robots.append(_make_robot(space, jx, jy, angle))

    ball = _make_ball(space, FIELD_W / 2, FIELD_H / 2)

    ctx = zmq.Context()
    pub = ctx.socket(zmq.PUB)
    pub.bind(f"tcp://*:{VISION_PORT}")

    pull = ctx.socket(zmq.PULL)
    pull.bind(f"tcp://*:{COMMAND_PORT}")
    pull.setsockopt(zmq.RCVTIMEO, 0)  # non-blocking

    commands: dict[str, list] = {str(i): [0.0, 0.0, 0.0] for i in range(NUM_ROBOTS)}
    pending_kicks: list[int] = []
    score = {"blue": 0, "red": 0}
    last_goal: dict | None = None
    goal_seq = 0
    sim_time = 0.0  # seconds of simulated physics, independent of wall clock

    print(
        f"[SimNode] world-state → :{VISION_PORT}   commands ← :{COMMAND_PORT}"
        f"   headless={args.headless}"
    )

    while True:
        t0 = time.perf_counter()

        # Drain pending wheel commands (non-blocking)
        while True:
            try:
                cmd = json.loads(pull.recv_string())
                ctype = cmd.get("type", "wheel")
                if ctype == "kick":
                    rid = int(cmd["robot_id"])
                    if 0 <= rid < NUM_ROBOTS:
                        pending_kicks.append(rid)
                    continue

                rid = str(cmd["robot_id"])
                if rid in commands and "wheel_speeds" in cmd:
                    commands[rid] = cmd["wheel_speeds"]
            except zmq.Again:
                break

        # Apply commands, damping, then advance physics
        for i, body in enumerate(robots):
            _apply_wheel_commands(body, commands[str(i)])
            _apply_damping(body, DT)

        if pending_kicks:
            for rid in pending_kicks:
                kicked = _try_kick_ball(robots[rid], ball)
                if kicked:
                    print(f"[SimNode] Kick by robot {rid}")
            pending_kicks.clear()

        _apply_damping_custom(ball, BALL_DAMP, BALL_DAMP, DT)

        space.step(DT)
        sim_time += DT

        scoring_team = _scoring_team(ball)
        if scoring_team is not None:
            score[scoring_team] += 1
            goal_seq += 1
            _reset_ball_to_center(ball)
            last_goal = {
                "seq": goal_seq,
                "team": scoring_team,
                "score": {"blue": score["blue"], "red": score["red"]},
                "t": sim_time,
            }
            print(
                f"[SimNode] GOAL {scoring_team.upper()}  "
                f"score {score['blue']}-{score['red']}"
            )

        # Publish world state
        state = {
            "t": sim_time,
            "score": {"blue": score["blue"], "red": score["red"]},
            "last_goal": last_goal,
            "ball": {
                "x": ball.position.x,
                "y": ball.position.y,
                "vx": ball.velocity.x,
                "vy": ball.velocity.y,
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
                for i, body in enumerate(robots)
            },
        }
        pub.send_string(json.dumps(state))

        if not args.headless:
            sleep_t = DT - (time.perf_counter() - t0)
            if sleep_t > 0:
                time.sleep(sleep_t)


if __name__ == "__main__":
    main()
