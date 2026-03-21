# receives world state, runs strategy & control the game
from __future__ import annotations
import argparse
import json
import numpy as np
import os
import sys

_PYTHON_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'python')
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _PYTHON_DIR)

from config import VISION_PORT, STRATEGY_PORT, STRATEGY_PORT_RED
from geometry import distance, lerp, opp_goal, our_goal
from prediction import predict_intercept_point
from state import GameState, build_game_state

OUR_COLOR = "blue"


class ZMQBackend:
    def __init__(self, color: str):
        import zmq
        self.zmq = zmq
        self.context = zmq.Context()

        self.vision_sub = self.context.socket(zmq.SUB)
        self.vision_sub.connect(f"tcp://localhost:{VISION_PORT}")
        self.vision_sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self.vision_sub.setsockopt(zmq.RCVTIMEO, 100)

        port = STRATEGY_PORT if color == "blue" else STRATEGY_PORT_RED
        self.strategy_pub = self.context.socket(zmq.PUB)
        self.strategy_pub.bind(f"tcp://*:{port}")

        print(f"[Strategy node] ZMQ mode: vision←:{VISION_PORT}  strategy→:{port}  color={color}")

    def receive_state(self) -> dict | None:
        try:
            raw = json.loads(self.vision_sub.recv_string())
        except self.zmq.Again:
            return None
        while True:
            try:
                raw = json.loads(self.vision_sub.recv_string(flags=self.zmq.NOBLOCK))
            except self.zmq.Again:
                break
        return raw

    def send_targets(self, msg: dict) -> None:
        self.strategy_pub.send_string(json.dumps(msg))


class TCPBackend:
    def __init__(self):
        import socket, time
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        for attempt in range(10):
            try:
                self.sock.connect(("127.0.0.1", VISION_PORT))
                print(f"[StrategyNode] TCP mode: connected to localhost:{VISION_PORT}")
                break
            except ConnectionRefusedError:
                print(f"[StrategyNode] Connection refused, retrying ({attempt + 1}/10)...")
                time.sleep(1)
        else:
            raise ConnectionError("Could not connect after 10 attempts")
        self.sock.setblocking(False)
        self.buffer = ""

    def receive_state(self) -> dict | None:
        import socket
        try:
            chunk = self.sock.recv(65536).decode()
            if not chunk:
                return None
            self.buffer += chunk
        except (BlockingIOError, socket.error):
            pass
        raw = None
        while "\n" in self.buffer:
            line, self.buffer = self.buffer.split("\n", 1)
            line = line.strip()
            if line:
                try:
                    raw = json.loads(line)
                except json.JSONDecodeError:
                    pass
        return raw

    def send_targets(self, msg: dict) -> None:
        try:
            self.sock.sendall((json.dumps(msg) + "\n").encode())
        except BrokenPipeError:
            print("[Strategy node] TCP mode: connection lost")


def decide(gamestate: GameState) -> dict[int, dict]:
    from BT.tree import tick
    return tick(gamestate, OUR_COLOR)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["tcp", "zmq"], default="zmq")
    parser.add_argument("--color", choices=["blue", "red"], default="blue")
    args = parser.parse_args()

    global OUR_COLOR
    OUR_COLOR = args.color

    if args.mode == "zmq":
        backend = ZMQBackend(color=OUR_COLOR)
    else:
        backend = TCPBackend()

    print(f"[Strategy node] started  mode={args.mode}  color={OUR_COLOR}")

    frame = 0
    while True:
        raw = backend.receive_state()
        if raw is None:
            continue

        try:
            gamestate = build_game_state(raw, OUR_COLOR)
            targets = decide(gamestate)
            msg = {
                "targets": {
                    str(rid): {
                        "x": float(t["pos"][0]),
                        "y": float(t["pos"][1]),
                        "kick": t["kick"],
                    }
                    for rid, t in targets.items()
                }
            }
            backend.send_targets(msg)
        except Exception as e:
            import traceback
            print(f"[ERROR] {e}")
            traceback.print_exc()
            continue

        frame += 1
        if frame % 300 == 0:
            print(f"[Strategy node] frame={frame}  possession={gamestate.possession}  ball=({gamestate.ball.pos[0]:.1f}, {gamestate.ball.pos[1]:.1f})")


if __name__ == "__main__":
    main()