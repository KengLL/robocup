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

from __future__ import annotations
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

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))  # project root (for decision_making.*)
sys.path.insert(0, _HERE)                   # python/ takes priority for config
from config import *

# collision types
# used for later collision logic
COLLISION_ROBOT = 1
COLLISION_BALL = 2
COLLISION_WALL = 3

# Goal mouth geometry is defined in config.py (GOAL_MOUTH_H, GOAL_Y_MIN,
# GOAL_Y_MAX) so viz and sim share one source of truth.

from decision_making.skills.kick import try_kick_ball

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

HALF_DURATION = 300.0  # seconds per half

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

    commands: dict[str, dict] = {
        str(i): {"wheel_speeds": [0.0, 0.0, 0.0], "kick": False}
        for i in range(NUM_ROBOTS)
    }
    pending_kicks: list[int] = []
    score = {"blue": 0, "red": 0}
    game_time = 0.0        # seconds elapsed in current half
    current_half = 1       # 1 or 2
    half_over = False
    game_over = False
    winner = None
    ball_stuck_timer = 0.0
    ball_stuck_seq = 0
    ball_last_pos = (FIELD_W / 2, FIELD_H / 2)
    BALL_STUCK_THRESHOLD = 0.02   #less than this = considered stuck
    BALL_STUCK_DURATION = 10.0     # seconds before ball goes back to middle
    game_phase = "FIRST HALF"
    last_goal: dict | None = None
    goal_seq = 0
    sim_time = 0.0  # seconds of simulated physics, independent of wall clock

    print(
        f"[SimNode] world-state → :{VISION_PORT}   commands ← :{COMMAND_PORT}"
        f"   headless={args.headless}"
    )
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
                ctype = cmd.get("type", "wheel")
                if ctype == "kick":
                    rid = int(cmd["robot_id"])
                    if 0 <= rid < NUM_ROBOTS:
                        pending_kicks.append(rid)
                    continue

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
            # 10 goal lead — end game immediately
            if abs(score["blue"] - score["red"]) >= 10:
                half_over = True
                game_phase = "FULL TIME"
                print(f"[SimNode] 10 GOAL LEAD — game over")
        
        if not half_over:
            game_time += DT
            if game_time >= HALF_DURATION:
                if current_half == 1:
                    current_half = 2
                    game_time = 0.0
                    game_phase = "SECOND HALF"
                    print("[SimNode] HALF TIME — starting second half")
                    _reset_ball_to_center(ball)
                elif current_half == 2:
                    if score["blue"] == score["red"]:
                        current_half = 3
                        game_time = 0.0
                        game_phase = "OVERTIME"
                        print("[SimNode] FULL TIME — scores equal, OVERTIME")
                        _reset_ball_to_center(ball)
                    else:
                        half_over = True
                        game_phase = "FULL TIME"
                        print(f"[SimNode] FULL TIME — Blue: {score['blue']}  Red: {score['red']}")
                elif current_half == 3:
                    current_half = 4
                    game_time = 0.0
                    game_phase = "OVERTIME 2ND"
                    print("[SimNode] OVERTIME second half")
                    _reset_ball_to_center(ball)
                elif current_half == 4:
                    half_over = True
                    game_phase = "FULL TIME"
                    print(f"[SimNode] OVERTIME FULL TIME — Blue: {score['blue']}  Red: {score['red']}")

        # ball stuck detection
        bx, by = ball.position.x, ball.position.y
        ball_moved = math.hypot(bx - ball_last_pos[0], by - ball_last_pos[1])
        if ball_moved < BALL_STUCK_THRESHOLD:
            ball_stuck_timer += DT
            if ball_stuck_timer >= BALL_STUCK_DURATION:
                print("[SimNode] Ball stuck — resetting to center")
                _reset_ball_to_center(ball)
                ball_stuck_timer = 0.0
                ball_last_pos = (FIELD_W / 2, FIELD_H / 2)
                ball_stuck_seq += 1
        else:
            ball_stuck_timer = 0.0
            ball_last_pos = (bx, by)
        # set winner whenever game ends
        if half_over and winner is None:
            if score["blue"] > score["red"]:
                winner = "blue"
            elif score["red"] > score["blue"]:
                winner = "red"
            else:
                winner = "draw"
            print(f"[SimNode] WINNER: {winner}")

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
            "t": sim_time,
            "score": {"blue": score["blue"], "red": score["red"]},
            "last_goal": last_goal,
            "game": {
                "half": current_half,
                "time_remaining": max(0.0, HALF_DURATION - game_time),
                "blue_score": score["blue"],
                "red_score": score["red"],
                "half_over": half_over,
                "phase": game_phase,
                "ball_stuck_seq": ball_stuck_seq,
                "winner": winner,
            },
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
