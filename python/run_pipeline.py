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

# (name, script, cwd)
NODES = [
    ("SimNode",      "simulation_node.py",            HERE),
    ("RobotNode",    "robot_node.py",                 HERE),
    ("VizNode",      "viz_node.py",                   HERE),
    ("StrategyBlue", "strategy_node.py --color blue", DECISION_DIR),
    #("StrategyRed",  "strategy_node.py --color red",  DECISION_DIR),
]

processes: list[subprocess.Popen] = []


def _launch(name: str, script: str, cwd: str) -> subprocess.Popen:
    parts = script.split()
    path = os.path.join(cwd, parts[0])
    p = subprocess.Popen([sys.executable, path] + parts[1:], cwd=cwd)
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
    _ = parser.add_argument(
        "--no-viz", action="store_true", help="Skip the Pygame visualization node"
    )
    _ = parser.add_argument(
        "--no-strategy", action="store_true", help="Skip the autonomous strategy node"
    )
    _ = parser.add_argument(
        "--color",
        choices=["blue", "red"],
        default="blue",
        help="Team color for the strategy node",
    )
    args = parser.parse_args()

    skip = set()
    if args.no_viz:
        skip.add("VizNode")

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    print("[Launcher] Starting RoboCup pipeline ...\n")

    nodes = [(n, s, d) for n, s, d in NODES if n not in skip]

    for name, script, cwd in nodes:
        p = _launch(name, script, cwd)
        processes.append(p)
        time.sleep(0.4)

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