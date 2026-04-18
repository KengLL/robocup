#!/usr/bin/env python3
"""
Pipeline Launcher — starts all nodes as separate subprocesses.

Usage:
    python run_pipeline.py
    python run_pipeline.py --no-viz
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
DECISION_DIR = os.path.join(os.path.dirname(HERE), "decision_making")

# Ordered start sequence: simulation must bind its sockets before others connect.
# (name, script, cwd)
NODES = [
    ("SimNode", "simulation_node.py", HERE),
    ("StrategyNode", "strategy_node.py", DECISION_DIR),
    ("RobotNode", "robot_node.py", HERE),
    ("VizNode", "viz_node.py", HERE),
]

POLICY_NODE = ("PolicyNode", "policy_node.py", HERE)

processes: list[subprocess.Popen] = []


def _launch(
    name: str, script: str, cwd: str, extra_args: list[str] | None = None
) -> subprocess.Popen:
    path = os.path.join(cwd, script)
    cmd = [sys.executable, path] + (extra_args or [])
    p = subprocess.Popen(cmd, cwd=cwd)
    print(f"  [{name}]  PID {p.pid:<6}  {script}")
    return p


def _shutdown(sig=None, frame=None) -> None:
    print("\n[Launcher] Shutting down all nodes ...")
    for p in processes:
        p.terminate()
    for p in processes:
        try:
            p.wait(timeout=3)
        except subprocess.TimeoutExpired:
            p.kill()
    print("[Launcher] Done.")
    sys.exit(0)


def main() -> None:
    parser = argparse.ArgumentParser(description="RoboCup Python Pipeline Launcher")
    parser.add_argument(
        "--no-viz", action="store_true", help="Skip the Pygame visualization node"
    )
    parser.add_argument(
        "--no-strategy", action="store_true", help="Skip the autonomous strategy node"
    )
    parser.add_argument(
        "--color",
        choices=["blue", "red"],
        default="blue",
        help="Team color for the strategy node",
    )
    parser.add_argument(
        "--policy",
        default=None,
        help=(
            "Path to a PPO checkpoint. When set, launches policy_node.py in "
            "place of strategy_node.py so the trained RL model drives the "
            "team through the same strategy→robot_node interface."
        ),
    )
    args = parser.parse_args()

    skip = set()
    if args.no_viz:
        skip.add("VizNode")
    if args.no_strategy or args.policy is not None:
        skip.add("StrategyNode")

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    print("[Launcher] Starting RoboCup pipeline ...\n")

    nodes = [(n, s, d) for n, s, d in NODES if n not in skip]
    if args.policy is not None:
        # Slot PolicyNode where StrategyNode would have been so the
        # PUB bind happens before RobotNode subscribes.
        insert_at = next(
            (i for i, (n, _, _) in enumerate(nodes) if n == "RobotNode"),
            len(nodes),
        )
        nodes.insert(insert_at, POLICY_NODE)

    for name, script, cwd in nodes:
        if name == "StrategyNode":
            extra = ["--mode", "zmq", "--color", args.color]
        elif name == "PolicyNode":
            extra = ["--checkpoint", args.policy]
        else:
            extra = None
        p = _launch(name, script, cwd, extra_args=extra)
        processes.append(p)
        time.sleep(0.4)  # stagger so PUB sockets bind before SUBs connect

    print(f"\n[Launcher] {len(nodes)} nodes running.  Ctrl+C to stop.\n")

    try:
        while True:
            for i, (name, _, _) in enumerate(nodes):
                ret = processes[i].poll()
                if ret is not None:
                    print(f"[Launcher] WARNING: {name} exited with code {ret}")
            time.sleep(2)
    except KeyboardInterrupt:
        _shutdown()


if __name__ == "__main__":
    main()
