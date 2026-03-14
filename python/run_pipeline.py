#!/usr/bin/env python3
"""
Pipeline Launcher — starts all nodes as separate subprocesses.

Usage:
    python run_pipeline.py
    python run_pipeline.py --no-viz
"""
import os, sys, subprocess, time, signal
import argparse

HERE = os.path.dirname(os.path.abspath(__file__))

# Ordered start sequence: simulation must bind its sockets before others connect.
NODES = [
    ("SimNode",   "simulation_node.py"),
    ("RobotNode", "robot_node.py"),
    ("VizNode",   "viz_node.py"),
]

processes: list[subprocess.Popen] = []


def _launch(name: str, script: str, extra_env: dict | None = None) -> subprocess.Popen:
    env  = os.environ.copy()
    if extra_env:
        env.update(extra_env)
    path = os.path.join(HERE, script)
    p    = subprocess.Popen([sys.executable, path], cwd=HERE, env=env)
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
    parser.add_argument("--no-viz", action="store_true",
                        help="Skip the Pygame visualization node")
    args = parser.parse_args()

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    print("[Launcher] Starting RoboCup pipeline ...\n")

    nodes = NODES if not args.no_viz else NODES[:-1]

    for name, script in nodes:
        p = _launch(name, script)
        processes.append(p)
        time.sleep(0.4)   # stagger so PUB sockets bind before SUBs connect

    print(f"\n[Launcher] {len(nodes)} nodes running.  Ctrl+C to stop.\n")

    try:
        while True:
            for i, (name, _) in enumerate(nodes):
                ret = processes[i].poll()
                if ret is not None:
                    print(f"[Launcher] WARNING: {name} exited with code {ret}")
            time.sleep(2)
    except KeyboardInterrupt:
        _shutdown()


if __name__ == "__main__":
    main()
