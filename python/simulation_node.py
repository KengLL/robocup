#!/usr/bin/env python3
"""
Simulation Node — ZMQ wrapper around :class:`PymunkWorld`.

Publishes world state on VISION_PORT (ZMQ PUB).
Receives robot wheel-speed commands on COMMAND_PORT (ZMQ PULL).

The physics engine lives in :mod:`pymunk_world`. This node owns the
transport and the gameplay clock (halves, overtime, ball-stuck reset).

In real-world deployment, swap this node for a Vision Node that reads
AprilTag data — the rest of the pipeline stays identical.

CLI flags:
  --headless   Do not sleep at the end of each tick. Physics advances
               as fast as CPU allows. Useful for CI.
  --seed INT   Seed Python's `random` and numpy RNGs. When set, initial
               robot positions are jittered by a small Gaussian.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from typing import Any

import zmq

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))
sys.path.insert(0, _HERE)
from config import COMMAND_PORT, DT, FIELD_H, FIELD_W, NUM_ROBOTS, VISION_PORT
from pymunk_world import PymunkWorld
from decision_making.skills.dribble import try_dribble_ball

HALF_DURATION = 300.0  # seconds per half

BALL_STUCK_THRESHOLD = 0.02  # ball moved less than this (m) per tick = stuck
BALL_STUCK_DURATION = 10.0  # seconds before ball is reset to center


def main() -> None:
    parser = argparse.ArgumentParser(description="RoboCup simulation node")
    _ = parser.add_argument(
        "--headless",
        action="store_true",
        help="Run the physics loop as fast as the CPU allows (no tick sleep).",
    )
    _ = parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Seed for random / numpy.random. Also enables Gaussian jitter "
        + "of initial robot positions for domain randomization.",
    )
    args = parser.parse_args()

    if args.seed is not None:
        print(f"[SimNode] seed = {args.seed}  (spawn jitter enabled)")

    world = PymunkWorld(seed=args.seed)

    ctx = zmq.Context()
    pub = ctx.socket(zmq.PUB)
    _ = pub.bind(f"tcp://*:{VISION_PORT}")

    pull = ctx.socket(zmq.PULL)
    _ = pull.bind(f"tcp://*:{COMMAND_PORT}")
    pull.setsockopt(zmq.RCVTIMEO, 0)  # non-blocking

    commands: dict[int, list[float]] = {i: [0.0, 0.0, 0.0] for i in range(NUM_ROBOTS)}
    pending_kicks: list[int] = []
    dribbling: set[int] = set()   # robot ids currently dribbling

    score = {"blue": 0, "red": 0}
    game_time = 0.0
    current_half = 1
    half_over = False
    winner: str | None = None
    game_phase = "FIRST HALF"
    last_goal: dict[str, Any] | None = None
    goal_seq = 0

    ball_stuck_timer = 0.0
    ball_stuck_seq = 0
    ball_last_pos = (FIELD_W / 2, FIELD_H / 2)

    print(
        f"[SimNode] world-state → :{VISION_PORT}   commands ← :{COMMAND_PORT}"
        + f"   headless={args.headless}"
    )

    while True:
        t0 = time.perf_counter()

        # Drain pending command messages (non-blocking).
        while True:
            try:
                cmd = json.loads(pull.recv_string())
                ctype = cmd.get("type", "wheel")
                if ctype == "kick":
                    rid = int(cmd["robot_id"])
                    if 0 <= rid < NUM_ROBOTS:
                        pending_kicks.append(rid)
                    continue
                if ctype == "dribble":
                    rid = int(cmd["robot_id"])
                    if cmd.get("active", False):
                        dribbling.add(rid)
                    else:
                        dribbling.discard(rid)
                    continue
                rid = int(cmd["robot_id"])
                if 0 <= rid < NUM_ROBOTS and "wheel_speeds" in cmd:
                    commands[rid] = cmd["wheel_speeds"]
            except zmq.Again:
                break

        state = world.step(commands, pending_kicks, dribbling)
        
        for rid in state["kicks"]:
            print(f"[SimNode] Kick by robot {rid}")
        pending_kicks.clear()

        # Scoring — increment counters; reset ball; optionally end the game.
        scoring_team = state["scoring_team"]
        if scoring_team is not None:
            score[scoring_team] += 1
            goal_seq += 1
            world.reset_ball()
            last_goal = {
                "seq": goal_seq,
                "team": scoring_team,
                "score": {"blue": score["blue"], "red": score["red"]},
                "t": state["t"],
            }
            print(
                f"[SimNode] GOAL {scoring_team.upper()}  "
                + f"score {score['blue']}-{score['red']}"
            )
            if abs(score["blue"] - score["red"]) >= 10:
                half_over = True
                game_phase = "FULL TIME"
                print("[SimNode] 10 GOAL LEAD — game over")

        # Game clock (halves + overtime).
        if not half_over:
            game_time += DT
            if game_time >= HALF_DURATION:
                if current_half == 1:
                    current_half = 2
                    game_time = 0.0
                    game_phase = "SECOND HALF"
                    print("[SimNode] HALF TIME — starting second half")
                    world.reset_ball()
                elif current_half == 2:
                    if score["blue"] == score["red"]:
                        current_half = 3
                        game_time = 0.0
                        game_phase = "OVERTIME"
                        print("[SimNode] FULL TIME — scores equal, OVERTIME")
                        world.reset_ball()
                    else:
                        half_over = True
                        game_phase = "FULL TIME"
                        print(
                            f"[SimNode] FULL TIME — Blue: {score['blue']}  "
                            + f"Red: {score['red']}"
                        )
                elif current_half == 3:
                    current_half = 4
                    game_time = 0.0
                    game_phase = "OVERTIME 2ND"
                    print("[SimNode] OVERTIME second half")
                    world.reset_ball()
                elif current_half == 4:
                    half_over = True
                    game_phase = "FULL TIME"
                    print(
                        f"[SimNode] OVERTIME FULL TIME — Blue: {score['blue']}  "
                        + f"Red: {score['red']}"
                    )

        # Ball-stuck detection.
        bx, by = state["ball"]["x"], state["ball"]["y"]
        ball_moved = math.hypot(bx - ball_last_pos[0], by - ball_last_pos[1])
        if ball_moved < BALL_STUCK_THRESHOLD:
            ball_stuck_timer += DT
            if ball_stuck_timer >= BALL_STUCK_DURATION:
                print("[SimNode] Ball stuck — resetting to center")
                world.reset_ball()
                ball_stuck_timer = 0.0
                ball_last_pos = (FIELD_W / 2, FIELD_H / 2)
                ball_stuck_seq += 1
        else:
            ball_stuck_timer = 0.0
            ball_last_pos = (bx, by)

        if half_over and winner is None:
            if score["blue"] > score["red"]:
                winner = "blue"
            elif score["red"] > score["blue"]:
                winner = "red"
            else:
                winner = "draw"
            print(f"[SimNode] WINNER: {winner}")

        # Publish world state in the historical JSON format.
        out = {
            "t": state["t"],
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
            "ball": state["ball"],
            "robots": state["robots"],
        }
        _ = pub.send_string(json.dumps(out))

        if not args.headless:
            sleep_t = DT - (time.perf_counter() - t0)
            if sleep_t > 0:
                time.sleep(sleep_t)


if __name__ == "__main__":
    main()
