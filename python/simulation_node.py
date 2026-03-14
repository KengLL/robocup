#!/usr/bin/env python3
"""
Simulation Node — Pymunk physics engine (replaces real camera/world).

Publishes world state on VISION_PORT (ZMQ PUB).
Receives robot wheel-speed commands on COMMAND_PORT (ZMQ PULL).

In real-world deployment, swap this node for a Vision Node that reads
AprilTag data — the rest of the pipeline stays identical.
"""
import os, sys, time, json, math
import pymunk
import zmq
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from config import *


def _make_robot(space: pymunk.Space, x: float, y: float, angle: float = 0.0) -> pymunk.Body:
    moment = pymunk.moment_for_circle(ROBOT_MASS, 0, ROBOT_RADIUS)
    body   = pymunk.Body(ROBOT_MASS, moment)
    body.position = (x, y)
    body.angle    = angle
    shape = pymunk.Circle(body, ROBOT_RADIUS)
    shape.elasticity = 0.3
    shape.friction   = 0.5
    space.add(body, shape)
    return body


def _add_walls(space: pymunk.Space) -> None:
    corners = [(0, 0), (FIELD_W, 0), (FIELD_W, FIELD_H), (0, FIELD_H)]
    for i in range(4):
        a, b = corners[i], corners[(i + 1) % 4]
        seg = pymunk.Segment(space.static_body, a, b, 0.02)
        seg.elasticity = 0.8
        seg.friction   = 0.5
        space.add(seg)


def _apply_damping(body: pymunk.Body, dt: float) -> None:
    """Manual per-step damping matching Godot RigidBody2D linear_damp."""
    lin_factor = max(0.0, 1.0 - LINEAR_DAMP  * dt)
    ang_factor = max(0.0, 1.0 - ANGULAR_DAMP * dt)
    body.velocity          = body.velocity * lin_factor
    body.angular_velocity *= ang_factor


def _apply_wheel_commands(body: pymunk.Body, wheel_speeds: list) -> None:
    """
    Forward kinematics: wheel speeds → forces on the pymunk body.
    Matches robot.gd _physics_process forward kinematics block.
    """
    total_force  = np.zeros(2)
    total_torque = 0.0
    for i, alpha in enumerate(WHEEL_ANGLES):
        force_mag   = wheel_speeds[i] * MOTOR_MAX_FORCE
        drive_angle = alpha + math.pi / 2.0
        total_force += np.array([math.cos(drive_angle), math.sin(drive_angle)]) * force_mag
        total_torque += force_mag * WHEEL_DISTANCE

    # Rotate local force vector to world frame (matches apply_central_force(rotated(rotation)))
    a  = body.angle
    wx = math.cos(a) * total_force[0] - math.sin(a) * total_force[1]
    wy = math.sin(a) * total_force[0] + math.cos(a) * total_force[1]
    body.apply_force_at_world_point((wx, wy), body.position)
    body.torque += total_torque


def main() -> None:
    space = pymunk.Space()
    space.gravity = (0, 0)
    _add_walls(space)

    robots = [
        _make_robot(space, 2.0, 3.0, 0.0),
    ]

    ctx  = zmq.Context()
    pub  = ctx.socket(zmq.PUB)
    pub.bind(f"tcp://*:{VISION_PORT}")

    pull = ctx.socket(zmq.PULL)
    pull.bind(f"tcp://*:{COMMAND_PORT}")
    pull.setsockopt(zmq.RCVTIMEO, 0)   # non-blocking

    commands: dict[str, list] = {str(i): [0.0, 0.0, 0.0] for i in range(NUM_ROBOTS)}

    print(f"[SimNode] world-state → :{VISION_PORT}   commands ← :{COMMAND_PORT}")

    while True:
        t0 = time.perf_counter()

        # Drain pending wheel commands (non-blocking)
        while True:
            try:
                cmd = json.loads(pull.recv_string())
                rid = str(cmd["robot_id"])
                if rid in commands:
                    commands[rid] = cmd["wheel_speeds"]
            except zmq.Again:
                break

        # Apply commands, damping, then advance physics
        for i, body in enumerate(robots):
            _apply_wheel_commands(body, commands[str(i)])
            _apply_damping(body, DT)
            
        space.step(DT)

        # Publish world state
        state = {
            "t": time.time(),
            "robots": {
                str(i): {
                    "x":  body.position.x, "y":  body.position.y,
                    "angle": body.angle,
                    "vx": body.velocity.x, "vy": body.velocity.y,
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
