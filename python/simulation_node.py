#!/usr/bin/env python3
"""
Simulation Node — ZMQ wrapper around :class:`PymunkWorld`.

Publishes world state on VISION_PORT (ZMQ PUB).
Receives robot wheel-speed commands on COMMAND_PORT (ZMQ PULL).

The physics engine lives in :mod:`pymunk_world`. This node owns the
transport and the gameplay clock (halves, overtime, ball-stuck reset).

With --vision, tracked objects are mirrored from the camera (see
vision_bridge.py); untracked robots stay simulated.

CLI flags:
  --headless   Do not sleep at the end of each tick. Physics advances
               as fast as CPU allows. Useful for CI.
  --seed INT   Seed Python's `random` and numpy RNGs. When set, initial
               robot positions are jittered by a small Gaussian.
  --vision SRC Mirror track-combined output: http://host:8000/state,
               tcp://host:5556 (ZMQ), or a recorded .jsonl to replay.
  --tag-map M  AprilTag id -> robot id, e.g. "0:0,4:1", or "auto" (default).
  --ignore-tags IDS  Tags never mirrored, e.g. "0,1,2,3" for corner tags.
  --flip-x / --flip-y  Mirror the camera field if motion comes out reversed.
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
from config import (
    COMMAND_PORT,
    DT,
    FIELD_H,
    FIELD_W,
    NUM_ROBOTS,
    VISION_FLIP_X,
    VISION_FLIP_Y,
    VISION_IGNORE_TAGS,
    VISION_PORT,
    VISION_TAG_MAP,
)
from pymunk_world import PymunkWorld
from vision_bridge import (
    FRESH,
    LOST,
    VisionBridge,
    apply_to_world,
    parse_ids,
    parse_tag_map,
)
from decision_making.skills.dribble import try_dribble_ball

HALF_DURATION = 300.0  # seconds per half
BALL_STUCK_THRESHOLD = 0.02
BALL_STUCK_DURATION = 10.0
VISION_LOG_PERIOD = 5.0  # seconds between vision stats prints

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
    _ = parser.add_argument(
        "--vision",
        default=None,
        help="Mirror camera tracking: http://host:port/state, tcp://host:port "
        + "(ZMQ), or a .jsonl log to replay.",
    )
    _ = parser.add_argument(
        "--tag-map",
        default=None,
        help='AprilTag id -> robot id, e.g. "0:0,4:1", or "auto": each new tag takes '
        + "the lowest free robot. Default: config.VISION_TAG_MAP (auto).",
    )
    _ = parser.add_argument(
        "--ignore-tags",
        default=None,
        help='Tag ids never mirrored, e.g. "0,1,2,3" for field-calibration corner tags.',
    )
    _ = parser.add_argument("--flip-x", action="store_true", default=VISION_FLIP_X)
    _ = parser.add_argument("--flip-y", action="store_true", default=VISION_FLIP_Y)
    _ = parser.add_argument(
        "--replay-speed",
        type=float,
        default=1.0,
        help="Playback speed when --vision is a .jsonl file.",
    )
    args = parser.parse_args()
    if args.vision and args.headless:
        parser.error("--vision runs in real time; drop --headless")

    if args.seed is not None:
        print(f"[SimNode] seed = {args.seed}  (spawn jitter enabled)")

    world = PymunkWorld(seed=args.seed)

    bridge: VisionBridge | None = None
    if args.vision:
        tag_map = parse_tag_map(args.tag_map) if args.tag_map else VISION_TAG_MAP
        ignore = parse_ids(args.ignore_tags) if args.ignore_tags else set(VISION_IGNORE_TAGS)
        bridge = VisionBridge(
            args.vision, tag_map, args.replay_speed, ignore, args.flip_x, args.flip_y
        ).start()
        world.set_mirrored(bridge.mirrored_rids)
        print(
            f"[SimNode] vision ← {args.vision} ({bridge.kind})  "
            + f"tags→robots {'auto' if tag_map is None else bridge.tag_map}"
            + (f"  ignoring tags {sorted(ignore)}" if ignore else "")
        )
    vision_log_t = time.perf_counter()

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
    ball_stuck_enabled = False  # toggled by viz via message
    ball_in_goal = False

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
                if ctype == "ball_stuck_toggle":
                    ball_stuck_enabled = not ball_stuck_enabled
                    print(f"[SimNode] Ball stuck reset → {'ON' if ball_stuck_enabled else 'OFF'}")
                    continue
                rid = int(cmd["robot_id"])
                if 0 <= rid < NUM_ROBOTS and "wheel_speeds" in cmd:
                    commands[rid] = cmd["wheel_speeds"]
            except zmq.Again:
                break

        vision_snap = None
        if bridge is not None:
            vision_snap = bridge.poll()
            if set(vision_snap["robots"]) != world.mirrored:
                world.set_mirrored(vision_snap["robots"])  # auto map claimed a robot
            apply_to_world(world, vision_snap)
        # Camera owns the ball: resets would just get snapped back next frame.
        vision_ball = vision_snap is not None and vision_snap["ball"].mode != LOST

        state = world.step(commands, pending_kicks, dribbling)
        
        for rid in state["kicks"]:
            print(f"[SimNode] Kick by robot {rid}")
        pending_kicks.clear()

        # Scoring — increment counters; reset ball; optionally end the game.
        # Edge-triggered so a mirrored ball sitting in the goal scores once.
        scoring_team = state["scoring_team"]
        if ball_in_goal:
            scoring_team = None
        ball_in_goal = state["scoring_team"] is not None
        if scoring_team is not None:
            score[scoring_team] += 1
            goal_seq += 1
            if not vision_ball:
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
        if ball_stuck_enabled and not vision_ball:
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
                "winner": winner,
                "ball_stuck_seq": ball_stuck_seq,
                "ball_stuck_enabled": ball_stuck_enabled,
            },
            "ball": state["ball"],
            "robots": state["robots"],
        }
        if vision_snap is not None:
            tags = vision_snap["stats"]["tags"]
            for rid, s in vision_snap["robots"].items():
                r = state["robots"][str(rid)]
                r["source"] = "vision"
                r["stale"] = s.mode != FRESH
                r["tag"] = tags.get(str(rid))
            b = vision_snap["ball"]
            state["ball"]["source"] = "sim" if b.mode == LOST else "vision"
            state["ball"]["stale"] = b.mode != FRESH
            out["vision"] = vision_snap["stats"]

            if time.perf_counter() - vision_log_t > VISION_LOG_PERIOD:
                vision_log_t = time.perf_counter()
                st = vision_snap["stats"]
                drops = "  ".join(
                    f"{k}:{'-' if v is None else f'{v:.0f}%'}" for k, v in st["drop_pct"].items()
                )
                print(
                    f"[SimNode] vision {'up' if st['link_up'] else 'DOWN'}  "
                    + f"{st['fps']:.1f} fps  lat {st['latency_ms']:.0f} ms  "
                    + f"link drops {st['transport_drops']}  rejects {st['rejects']}  "
                    + f"unseen {drops}"
                )
        _ = pub.send_string(json.dumps(out))

        if not args.headless:
            sleep_t = DT - (time.perf_counter() - t0)
            if sleep_t > 0:
                time.sleep(sleep_t)


if __name__ == "__main__":
    main()
