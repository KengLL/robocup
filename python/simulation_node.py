#!/usr/bin/env python3
"""
Simulation Node — Pymunk physics engine (replaces real camera/world).

Publishes world state on VISION_PORT (ZMQ PUB).
Receives robot wheel-speed commands on COMMAND_PORT (ZMQ PULL).

In real-world deployment, swap this node for a Vision Node that reads
AprilTag data — the rest of the pipeline stays identical.
"""

import json
import math
import os
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

#goal positions
GOAL_WIDTH = 1.0
GOAL_Y_MIN = FIELD_H / 2 - GOAL_WIDTH / 2
GOAL_Y_MAX = FIELD_H / 2 + GOAL_WIDTH / 2

#reset the ball posiitons and robots
def _reset_positions(robots, ball):
    ball.position = (FIELD_W / 2, FIELD_H / 2)
    ball.velocity = (0, 0)
    ball.angular_velocity = 0
    start_positions = [
        (2.0, 3.0, 0.0),
        (1.5, 4.5, 0.0),
        (0.8, 3.0, 0.0),
        (7.0, 3.0, math.pi),
        (7.5, 1.5, math.pi),
        (8.2, 3.0, math.pi),
    ]
    for body, (x, y, angle) in zip(robots, start_positions):
        body.position = (x, y)
        body.velocity = (0, 0)
        body.angle = angle
        body.angular_velocity = 0


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

def _add_walls(space: pymunk.Space) -> None:
    # bottom wall
    seg = pymunk.Segment(space.static_body, (0, 0), (FIELD_W, 0), 0.02)
    seg.elasticity = 0.8
    seg.friction = 0.5
    space.add(seg)

    # top wall
    seg = pymunk.Segment(space.static_body, (0, FIELD_H), (FIELD_W, FIELD_H), 0.02)
    seg.elasticity = 0.8
    seg.friction = 0.5
    space.add(seg)

    # left wall — two segments with gap for goal
    seg = pymunk.Segment(space.static_body, (0, 0), (0, GOAL_Y_MIN), 0.02)
    seg.elasticity = 0.8
    seg.friction = 0.5
    space.add(seg)
    seg = pymunk.Segment(space.static_body, (0, GOAL_Y_MAX), (0, FIELD_H), 0.02)
    seg.elasticity = 0.8
    seg.friction = 0.5
    space.add(seg)

    # right wall — two segments with gap for goal
    seg = pymunk.Segment(space.static_body, (FIELD_W, 0), (FIELD_W, GOAL_Y_MIN), 0.02)
    seg.elasticity = 0.8
    seg.friction = 0.5
    space.add(seg)
    seg = pymunk.Segment(space.static_body, (FIELD_W, GOAL_Y_MAX), (FIELD_W, FIELD_H), 0.02)
    seg.elasticity = 0.8
    seg.friction = 0.5
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


def main() -> None:
    space = pymunk.Space()
    space.gravity = (0, 0)
    _add_walls(space)

    # blue team (robots 0-2) — left side
    # red team (robots 3-5) — right side
    robots = [
        # Blue team
        _make_robot(space, 2.0, 3.0, 0.0),  # 0: blue attacker
        _make_robot(space, 1.5, 4.5, 0.0),  # 1: blue supporter
        _make_robot(space, 0.8, 3.0, 0.0),  # 2: blue defender
        # Red team
        _make_robot(space, 7.0, 3.0, math.pi),  # 3: red attacker
        _make_robot(space, 7.5, 1.5, math.pi),  # 4: red supporter
        _make_robot(space, 8.2, 3.0, math.pi),  # 5: red defender
    ]

    ball = _make_ball(space, FIELD_W / 2, FIELD_H / 2)

    ctx = zmq.Context()
    pub = ctx.socket(zmq.PUB)
    pub.bind(f"tcp://*:{VISION_PORT}")

    pull = ctx.socket(zmq.PULL)
    pull.bind(f"tcp://*:{COMMAND_PORT}")
    pull.setsockopt(zmq.RCVTIMEO, 0)  # non-blocking

    commands: dict[str, dict] = {
        str(i): {"wheel_speeds": [0.0, 0.0, 0.0], "kick": False}
        for i in range(NUM_ROBOTS)
    }

    blue_score = 0
    red_score = 0


    print(f"[SimNode] world-state → :{VISION_PORT}   commands ← :{COMMAND_PORT}")

    while True:
        t0 = time.perf_counter()

        # Drain pending wheel commands (non-blocking)
        while True:
            try:
                cmd = json.loads(pull.recv_string())
                rid = str(cmd["robot_id"])
                if rid in commands:
                    commands[rid] = {
                        "wheel_speeds": cmd["wheel_speeds"],
                        "kick": cmd.get("kick", False),
                    }
            except zmq.Again:
                break

        # Apply commands, damping, then advance physics
        for i, body in enumerate(robots):
            _apply_wheel_commands(body, commands[str(i)]["wheel_speeds"])
            _apply_damping(body, DT)

        #kicking
        KICK_IMPULSE = 5.0
        KICK_RANGE = 0.15
        for i, body in enumerate(robots):
            if commands[str(i)]["kick"]:
                dx = ball.position.x - body.position.x
                dy = ball.position.y - body.position.y
                dist = math.sqrt(dx**2 + dy**2)
                if dist < KICK_RANGE:
                    nx, ny = dx / (dist + 1e-6), dy / (dist + 1e-6)
                    ball.apply_impulse_at_world_point(
                        (nx * KICK_IMPULSE, ny * KICK_IMPULSE),
                        ball.position
                    )
        commands[str(i)]["kick"] = False  # reset after one frame

        space.step(DT)

        bx = ball.position.x
        by = ball.position.y
        if GOAL_Y_MIN <= by <= GOAL_Y_MAX:
            if bx <= 0.0:
                red_score += 1
                print(f"[SimNode] GOAL for RED! Score — Blue: {blue_score}  Red: {red_score}")
                _reset_positions(robots, ball)
            elif bx >= FIELD_W:
                blue_score += 1
                print(f"[SimNode] GOAL for BLUE! Score — Blue: {blue_score}  Red: {red_score}")
                _reset_positions(robots, ball)

        # Publish world state
        state = {
            "t": time.time(),
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

        sleep_t = DT - (time.perf_counter() - t0)
        if sleep_t > 0:
            time.sleep(sleep_t)


if __name__ == "__main__":
    main()
