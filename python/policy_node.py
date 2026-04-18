"""
Policy Node — PPO-trained strategy layer, drop-in for strategy_node.

Reads world state from VISION_PORT, runs the policy network, and
publishes {targets, kicks} on STRATEGY_PORT. robot_node receives those
targets exactly like the rule-based strategy_node's output, runs its
low-level dynamic-inversion controller, and pushes wheel commands to
the simulator. Key 5 in viz_node toggles autonomous mode on/off the
same way it did for the rule-based strategy.

Usage::

    python policy_node.py --checkpoint checkpoints/ppo_targets_1M.pt

Runs indefinitely. Ctrl+C to stop.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np
import torch
import zmq

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from config import FIELD_H, FIELD_W, NUM_ROBOTS, STRATEGY_PORT, VISION_PORT
from rl_env import ACT_DIM, N_OURS, OBS_DIM
from train_ppo import ActorCritic, RunningMeanStd


def flatten_vision(state: dict) -> np.ndarray:
    """Match rl_env.RoboCupRLEnv._flatten so the trained network sees the
    same feature layout it was trained on."""
    out = np.zeros(OBS_DIM, dtype=np.float32)
    b = state["ball"]
    out[0] = (b["x"] - FIELD_W / 2.0) / (FIELD_W / 2.0)
    out[1] = (b["y"] - FIELD_H / 2.0) / (FIELD_H / 2.0)
    out[2] = b["vx"]
    out[3] = b["vy"]
    for rid in range(NUM_ROBOTS):
        r = state["robots"][str(rid)]
        base = 4 + rid * 7
        out[base + 0] = (r["x"] - FIELD_W / 2.0) / (FIELD_W / 2.0)
        out[base + 1] = (r["y"] - FIELD_H / 2.0) / (FIELD_H / 2.0)
        out[base + 2] = math.cos(r["angle"])
        out[base + 3] = math.sin(r["angle"])
        out[base + 4] = r["vx"]
        out[base + 5] = r["vy"]
        out[base + 6] = r.get("omega", 0.0)
    return out


def decode_action(a: np.ndarray) -> tuple[dict[str, dict], list[int]]:
    """Mirror of rl_env._decode_action, but in target-publishing form."""
    targets: dict[str, dict] = {}
    kicks: list[int] = []
    for i in range(N_OURS):
        base = i * 4
        tx_norm = float(a[base + 0])
        ty_norm = float(a[base + 1])
        ttheta_norm = float(a[base + 2])
        kick = float(a[base + 3])
        targets[str(i)] = {
            "x": (tx_norm + 1.0) / 2.0 * FIELD_W,
            "y": (ty_norm + 1.0) / 2.0 * FIELD_H,
            "theta": ttheta_norm * math.pi,
            "mode": "2005_INVERSION",
        }
        if kick > 0.0:
            kicks.append(i)
    return targets, kicks


def main() -> None:
    parser = argparse.ArgumentParser(description="PPO policy node")
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--deterministic", action="store_true", default=True)
    parser.add_argument(
        "--stochastic", action="store_false", dest="deterministic",
        help="Sample from the policy distribution instead of using the mean."
    )
    args = parser.parse_args()

    ckpt = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
    model = ActorCritic(OBS_DIM, ACT_DIM)
    model.load_state_dict(ckpt["model_state_dict"])
    model.eval()

    rms = RunningMeanStd((OBS_DIM,))
    rms.mean = np.asarray(ckpt["obs_rms_mean"])
    rms.var = np.asarray(ckpt["obs_rms_var"])
    rms.count = float(ckpt["obs_rms_count"])

    ctx = zmq.Context()
    vision_sub = ctx.socket(zmq.SUB)
    vision_sub.connect(f"tcp://localhost:{VISION_PORT}")
    vision_sub.setsockopt_string(zmq.SUBSCRIBE, "")
    vision_sub.setsockopt(zmq.RCVTIMEO, 200)

    strategy_pub = ctx.socket(zmq.PUB)
    strategy_pub.bind(f"tcp://*:{STRATEGY_PORT}")

    print(
        f"[PolicyNode] checkpoint={os.path.basename(args.checkpoint)}  "
        f"vision←:{VISION_PORT}  targets→:{STRATEGY_PORT}  "
        f"deterministic={args.deterministic}"
    )

    frame = 0
    while True:
        try:
            raw = vision_sub.recv_string()
        except zmq.Again:
            continue

        # Drain backlog so we always act on the most recent frame.
        while True:
            try:
                raw = vision_sub.recv_string(flags=zmq.NOBLOCK)
            except zmq.Again:
                break

        try:
            state = json.loads(raw)
        except json.JSONDecodeError:
            continue

        obs = flatten_vision(state)
        obs_n = rms.normalize(obs[None, :])
        with torch.no_grad():
            action, _, _ = model.act(
                torch.as_tensor(obs_n, dtype=torch.float32),
                deterministic=args.deterministic,
            )
        a = action.squeeze(0).numpy()
        targets, kicks = decode_action(a)

        strategy_pub.send_string(json.dumps({
            "targets": targets,
            "kicks": kicks,
        }))

        frame += 1
        if frame % 60 == 0:
            print(
                f"[PolicyNode] frame {frame}  "
                f"ball=({state['ball']['x']:.2f}, {state['ball']['y']:.2f})  "
                f"score={state['score']['blue']}-{state['score']['red']}"
            )


if __name__ == "__main__":
    main()
