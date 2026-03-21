# receives world state, runs strategy & control the game
import argparse
import json
import os
import sys

import numpy as np
from geometry import distance, lerp, opp_goal, our_goal
from prediction import predict_intercept_point
from state import GameState, build_game_state

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

VISION_PORT = 9090
STRATEGY_PORT = 9091

OUR_COLOR = "blue"


class ZMQBackend:
    def __init__(self):
        import zmq

        self.zmq = zmq
        self.context = zmq.Context()

        self.vision_sub = self.context.socket(zmq.SUB)
        self.vision_sub.connect(f"tcp://localhost:{VISION_PORT}")
        self.vision_sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self.vision_sub.setsockopt(zmq.RCVTIMEO, 100)

        self.strategy_pub = self.context.socket(zmq.PUB)
        self.strategy_pub.bind(f"tcp://*:{STRATEGY_PORT}")

        print(
            f"[Strategy node] ZMQ mode: connected to vision on port {VISION_PORT} and strategy on port {STRATEGY_PORT}"
        )

    def receive_state(self) -> dict | None:
        raw = None
        while True:
            try:
                raw = json.loads(self.vision_sub.recv_string())
            except self.zmq.Again:
                break
        return raw

    def send_targets(self, msg: dict) -> None:
        self.strategy_pub.send_string(json.dumps(msg))


class TCPBackend:
    def __init__(self):
        import socket
        import time

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        for attempt in range(10):
            try:
                self.sock.connect(("127.0.0.1", VISION_PORT))
                print(f"[StrategyNode] TCP mode: connected to localhost:{VISION_PORT}")
                break
            except ConnectionRefusedError:
                print(
                    f"[StrategyNode] Connection refused, retrying ({attempt + 1}/10)..."
                )
                time.sleep(1)
        else:
            raise ConnectionError("Could not connect to Godot after 10 attempts")

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


def decide(gamestate: GameState) -> dict[int, np.ndarray]:
    # Placeholder for BT
    # Closest Robot: attacker, intercepts ball
    # Second Closest: defender, offset towards opponent goal
    # Third Closest: covers own goal

    targets = {}
    ball = gamestate.ball
    robots = sorted(gamestate.our_team, key=lambda r: distance(r.pos, ball.pos))

    for i, robot in enumerate(robots):
        if i == 0:
            targets[robot.id] = predict_intercept_point(ball, robot, 0.9, 2.0, 20)
        elif i == 1:
            goal = opp_goal(OUR_COLOR)
            to_goal = (goal - ball.pos) / (np.linalg.norm(goal - ball.pos) + 1e-6)
            perp = np.array([-to_goal[1], to_goal[0]])
            targets[robot.id] = ball.pos + to_goal * 1.5 + perp * 1.0
        else:
            goal = our_goal(OUR_COLOR)
            targets[robot.id] = lerp(goal, ball.pos, 0.3)
    return targets


def main() -> None:
    parser = argparse.ArgumentParser(description="Strategy node for the RoboCup game")
    parser.add_argument(
        "--mode", choices=["tcp", "zmq"], default="tcp", help="Mode: tcp or zmq"
    )
    parser.add_argument(
        "--color",
        choices=["blue", "red"],
        default="blue",
        help="Team color: blue or red",
    )
    args = parser.parse_args()

    global OUR_COLOR
    OUR_COLOR = args.color

    backend = ZMQBackend() if args.mode == "zmq" else TCPBackend()

    print(f"[Strategy node] {args.mode} mode: started")
    print(f"[Strategy node] {args.mode} mode: Controlling {OUR_COLOR} team")

    frame = 0
    while True:
        raw = backend.receive_state()
        if raw is None:
            continue
        gamestate = build_game_state(raw, OUR_COLOR)

        targets = decide(gamestate)

        msg = {
            "targets": {
                str(rid): {
                    "x": float(pos[0]),
                    "y": float(pos[1]),
                }
                for rid, pos in targets.items()
            }
        }

        backend.send_targets(msg)
        frame += 1

        if frame % 300 == 0:
            print(f"[Strategy node] {args.mode} mode: frame {frame}")
            print(f"[Strategy node] possession = {gamestate.possession}")
            print(f"ball=({gamestate.ball.pos[0]:.1f}, {gamestate.ball.pos[1]:.1f})")


if __name__ == "__main__":
    main()
