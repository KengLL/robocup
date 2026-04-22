# receives world state, runs strategy & control the game
from __future__ import annotations

import argparse
import json
import os
import sys
from typing import TYPE_CHECKING, Any, final

import numpy as np

if TYPE_CHECKING:
    from decision_making.skills.rl_kick import RLKickSkill

# Add project root so `decision_making.*` resolves when run as a standalone script
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

VISION_PORT = 9090
STRATEGY_PORT = 9091
COMMAND_PORT = 9092
MANUAL_PORT = 9093

DEFAULT_RL_CHECKPOINT = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "rl", "checkpoints", "first_final", "final.zip",
)

from decision_making.geometry import distance, lerp, opp_goal, our_goal  # noqa: E402
from decision_making.prediction import predict_intercept_point  # noqa: E402
from decision_making.state import (  # noqa: E402
    FIELD_LENGTH,
    GameState,
    build_game_state,
)

our_color = "blue"


@final
class ZMQBackend:
    def __init__(self):
        import zmq

        self.zmq = zmq
        self.context = zmq.Context()

        self.vision_sub = self.context.socket(zmq.SUB)
        _ = self.vision_sub.connect(f"tcp://localhost:{VISION_PORT}")
        self.vision_sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self.vision_sub.setsockopt(zmq.RCVTIMEO, 100)

        self.strategy_pub = self.context.socket(zmq.PUB)
        _ = self.strategy_pub.bind(f"tcp://*:{STRATEGY_PORT}")

        # Listen to viz for the RL-kick toggle.
        self.manual_sub = self.context.socket(zmq.SUB)
        _ = self.manual_sub.connect(f"tcp://localhost:{MANUAL_PORT}")
        self.manual_sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self.manual_sub.setsockopt(zmq.RCVTIMEO, 0)  # non-blocking

        # Push kick commands straight to the sim.
        self.cmd_push = self.context.socket(zmq.PUSH)
        _ = self.cmd_push.connect(f"tcp://localhost:{COMMAND_PORT}")

        print(
            f"[Strategy node] ZMQ mode: vision←:{VISION_PORT}  "
            + f"strategy→:{STRATEGY_PORT}  manual←:{MANUAL_PORT}  cmds→:{COMMAND_PORT}"
        )

    def receive_state(self) -> dict[str, Any] | None:
        try:
            raw = json.loads(self.vision_sub.recv_string())
        except self.zmq.Again:
            return None
        # Drain buffered frames so we always process the most recent state.
        while True:
            try:
                raw = json.loads(self.vision_sub.recv_string(flags=self.zmq.NOBLOCK))
            except self.zmq.Again:
                break
        return raw

    def drain_manual(self) -> list[dict[str, Any]]:
        msgs: list[dict[str, Any]] = []
        while True:
            try:
                msgs.append(json.loads(self.manual_sub.recv_string()))
            except self.zmq.Again:
                break
        return msgs

    def send_targets(self, msg: dict[str, Any]) -> None:
        _ = self.strategy_pub.send_string(json.dumps(msg))

    def send_kick(self, robot_id: int) -> None:
        _ = self.cmd_push.send_string(
            json.dumps({"type": "kick", "robot_id": int(robot_id)})
        )


@final
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

    def receive_state(self) -> dict[str, Any] | None:
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

    def drain_manual(self) -> list[dict[str, Any]]:
        return []

    def send_targets(self, msg: dict[str, Any]) -> None:
        try:
            self.sock.sendall((json.dumps(msg) + "\n").encode())
        except BrokenPipeError:
            print("[Strategy node] TCP mode: connection lost")

    def send_kick(self, robot_id: int) -> None:
        pass  # TCP mode has no command channel; kicks only work in ZMQ mode.


def decide(
    gamestate: GameState,
    rl_skill: "RLKickSkill | None" = None,
    rl_enabled: bool = False,
) -> tuple[dict[int, np.ndarray], list[int]]:
    """Return (targets, kick_rids).

    When rl_skill is loaded and rl_enabled is on and the ball is in the
    opponent half, the attacker's target comes from the RL policy and a
    kick is emitted every tick (the zone gate in try_kick_ball filters).
    """
    targets: dict[int, np.ndarray] = {}
    kicks: list[int] = []
    ball = gamestate.ball
    robots = sorted(gamestate.our_team, key=lambda r: distance(r.pos, ball.pos))

    ball_in_opp_half = _ball_in_opp_half(ball.pos[0])

    for i, robot in enumerate(robots):
        if i == 0:
            if rl_enabled and rl_skill is not None and ball_in_opp_half:
                tx, ty = rl_skill.target(robot, ball)
                targets[robot.id] = np.array([tx, ty])
                kicks.append(robot.id)
            else:
                targets[robot.id] = predict_intercept_point(ball, robot, 0.9, 2.0, 20)
        elif i == 1:
            goal = opp_goal(our_color)
            to_goal = (goal - ball.pos) / (np.linalg.norm(goal - ball.pos) + 1e-6)
            perp = np.array([-to_goal[1], to_goal[0]])
            targets[robot.id] = ball.pos + to_goal * 1.5 + perp * 1.0
        else:
            goal = our_goal(our_color)
            targets[robot.id] = lerp(goal, ball.pos, 0.3)
    return targets, kicks


def _ball_in_opp_half(ball_x: float) -> bool:
    # Blue-attacks-right: opp half is the right half of the field.
    if our_color == "blue":
        return ball_x > FIELD_LENGTH / 2.0
    return ball_x < FIELD_LENGTH / 2.0


def main() -> None:
    parser = argparse.ArgumentParser(description="Strategy node for the RoboCup game")
    _ = parser.add_argument(
        "--mode", choices=["tcp", "zmq"], default="zmq", help="Mode: tcp or zmq"
    )
    _ = parser.add_argument(
        "--color",
        choices=["blue", "red"],
        default="blue",
        help="Team color: blue or red",
    )
    _ = parser.add_argument(
        "--rl-checkpoint",
        type=str,
        default=DEFAULT_RL_CHECKPOINT,
        help="Path to the trained PPO kick checkpoint. Loaded lazily on first toggle-on.",
    )
    args = parser.parse_args()

    global our_color
    our_color = args.color

    backend = ZMQBackend() if args.mode == "zmq" else TCPBackend()

    print(f"[Strategy node] {args.mode} mode: started")
    print(f"[Strategy node] {args.mode} mode: controlling {our_color} team")

    rl_skill = None
    rl_enabled = False

    frame = 0
    while True:
        raw = backend.receive_state()
        if raw is None:
            continue

        # Handle manual-port control messages (RL toggle from viz).
        for msg in backend.drain_manual():
            if "rl_kick_enabled" in msg:
                want = bool(msg["rl_kick_enabled"])
                if want and rl_skill is None:
                    rl_skill = _try_load_skill(args.rl_checkpoint)
                rl_enabled = want and rl_skill is not None
                print(f"[Strategy node] RL kick → {'ON' if rl_enabled else 'OFF'}")

        try:
            gamestate = build_game_state(raw, our_color)
            targets, kicks = decide(gamestate, rl_skill, rl_enabled)
            backend.send_targets({
                "targets": {
                    str(rid): {"x": float(pos[0]), "y": float(pos[1])}
                    for rid, pos in targets.items()
                }
            })
            for rid in kicks:
                backend.send_kick(rid)
        except Exception as e:
            print(f"[ERROR] {e}")
            import traceback
            traceback.print_exc()
            continue

        frame += 1
        if frame % 300 == 0:
            print(
                f"[Strategy node] frame={frame}  possession={gamestate.possession}  "
                + f"rl_kick={'ON' if rl_enabled else 'OFF'}  "
                + f"ball=({gamestate.ball.pos[0]:.1f}, {gamestate.ball.pos[1]:.1f})"
            )


def _try_load_skill(path: str):
    if not os.path.exists(path):
        print(f"[Strategy node] RL checkpoint not found: {path}")
        return None
    try:
        from decision_making.skills.rl_kick import RLKickSkill
        skill = RLKickSkill(path)
        print(f"[Strategy node] loaded RL kick from {path}")
        return skill
    except Exception as e:
        print(f"[Strategy node] failed to load RL kick: {e}")
        return None


if __name__ == "__main__":
    main()
