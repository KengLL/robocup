# receives world state, runs strategy & controls the game
from __future__ import annotations

import argparse
import glob
import json
import os
import sys
from typing import Any, final

import numpy as np

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(THIS_DIR)
PYTHON_DIR = os.path.join(PROJECT_ROOT, "python")

sys.path.insert(0, THIS_DIR)
sys.path.insert(0, PYTHON_DIR)
sys.path.insert(0, PROJECT_ROOT)

from BT.attacker_tree import load_rl_skill, set_rl_enabled  # noqa: E402
from geometry import distance, our_goal  # noqa: E402
from state import GameState, RobotState, build_game_state  # noqa: E402


VISION_PORT = 9090
STRATEGY_PORT = 9091
COMMAND_PORT = 9092
MANUAL_PORT = 9093

DEFAULT_RL_CHECKPOINT = os.path.join(
    THIS_DIR,
    "rl",
    "checkpoints",
    "first_final",
    "final.zip",
)

_ckpt_dir = os.path.join(THIS_DIR, "rl", "checkpoints")
_ckpts = sorted(glob.glob(os.path.join(_ckpt_dir, "**", "final.zip"), recursive=True))
_rl_tree_loaded = False
if _ckpts:
    _rl_tree_loaded = load_rl_skill(_ckpts[-1])

our_color = "blue"


@final
class ZMQBackend:
    def __init__(self, color: str):
        import zmq

        self.zmq = zmq
        self.context = zmq.Context()

        self.vision_sub = self.context.socket(zmq.SUB)
        self.vision_sub.connect(f"tcp://localhost:{VISION_PORT}")
        self.vision_sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self.vision_sub.setsockopt(zmq.RCVTIMEO, 100)

        self.strategy_pub = self.context.socket(zmq.PUB)
        self.strategy_pub.bind(f"tcp://*:{STRATEGY_PORT}")

        self.manual_sub = self.context.socket(zmq.SUB)
        self.manual_sub.connect(f"tcp://localhost:{MANUAL_PORT}")
        self.manual_sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self.manual_sub.setsockopt(zmq.RCVTIMEO, 0)

        self.cmd_push = self.context.socket(zmq.PUSH)
        self.cmd_push.connect(f"tcp://localhost:{COMMAND_PORT}")

        print(
            f"[Strategy node] ZMQ mode: vision←:{VISION_PORT}  "
            + f"strategy→:{STRATEGY_PORT}  manual←:{MANUAL_PORT}  cmds→:{COMMAND_PORT}"
        )

    def receive_state(self) -> dict[str, Any] | None:
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

    def drain_manual(self) -> list[dict[str, Any]]:
        msgs: list[dict[str, Any]] = []
        while True:
            try:
                msgs.append(json.loads(self.manual_sub.recv_string()))
            except self.zmq.Again:
                break
        return msgs

    def send_targets(self, msg: dict[str, Any]) -> None:
        self.strategy_pub.send_string(json.dumps(msg))

    def send_kick(self, robot_id: int) -> None:
        self.cmd_push.send_string(
            json.dumps({"type": "kick", "robot_id": int(robot_id)})
        )

    def send_dribble(self, robot_id: int, active: bool) -> None:
        self.cmd_push.send_string(
            json.dumps(
                {"type": "dribble", "robot_id": int(robot_id), "active": bool(active)}
            )
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
                print(f"[StrategyNode] Connection refused, retrying ({attempt + 1}/10)...")
                time.sleep(1)
        else:
            raise ConnectionError("Could not connect after 10 attempts")
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
        pass

    def send_dribble(self, robot_id: int, active: bool) -> None:
        pass


def _assign_roles(gamestate: GameState, color: str) -> dict[str, RobotState]:
    goal = our_goal(color)
    robots = list(gamestate.our_team)

    if not robots:
        return {}

    attacker = min(robots, key=lambda r: distance(r.pos, gamestate.ball.pos))
    remaining = [r for r in robots if r.id != attacker.id]
    roles: dict[str, RobotState] = {"attacker": attacker}

    if remaining:
        defender = min(remaining, key=lambda r: distance(r.pos, goal))
        roles["defender"] = defender
        remaining = [r for r in remaining if r.id != defender.id]

    if remaining:
        roles["supporter"] = remaining[0]

    return roles


def _tick_team(gamestate: GameState, color: str) -> tuple[dict[int, dict], set[int]]:
    from BT.attacker_tree import attacker_decide
    from BT.defender_tree import defender_decide
    from BT.supporter_tree import supporter_decide

    roles = _assign_roles(gamestate, color)
    targets: dict[int, dict] = {}
    attacker_ids: set[int] = set()

    attacker = roles.get("attacker")
    if attacker is not None:
        attacker_ids.add(attacker.id)
        pos, kick, angle, dribble = attacker_decide(attacker, gamestate, color)
        targets[attacker.id] = {
            "pos": pos,
            "kick": kick,
            "angle": angle,
            "dribble": dribble,
        }

    supporter = roles.get("supporter")
    if supporter is not None:
        pos, kick = supporter_decide(supporter, gamestate, color)
        targets[supporter.id] = {
            "pos": pos,
            "kick": kick,
            "angle": None,
            "dribble": False,
        }

    defender = roles.get("defender")
    if defender is not None:
        pos, kick = defender_decide(defender, gamestate, color)
        targets[defender.id] = {
            "pos": pos,
            "kick": kick,
            "angle": None,
            "dribble": False,
        }

    return targets, attacker_ids


def decide(gamestate: GameState) -> tuple[dict[int, dict], set[int]]:
    all_targets: dict[int, dict] = {}
    attacker_ids: set[int] = set()

    for color, our_team, opponents in (
        ("blue", gamestate.blue, gamestate.red),
        ("red", gamestate.red, gamestate.blue),
    ):
        original_our_team = getattr(gamestate, "our_team", None)
        original_opponents_team = getattr(gamestate, "opponents_team", None)
        object.__setattr__(gamestate, "our_team", our_team)
        object.__setattr__(gamestate, "opponents_team", opponents)

        try:
            team_targets, team_attackers = _tick_team(gamestate, color)
            all_targets.update(team_targets)
            attacker_ids.update(team_attackers)
        finally:
            if original_our_team is not None:
                object.__setattr__(gamestate, "our_team", original_our_team)
            if original_opponents_team is not None:
                object.__setattr__(gamestate, "opponents_team", original_opponents_team)

    return all_targets, attacker_ids


def _pos_to_xy(pos: np.ndarray) -> tuple[float, float]:
    return float(pos[0]), float(pos[1])


def main() -> None:
    parser = argparse.ArgumentParser(description="Strategy node for the RoboCup game")
    parser.add_argument(
        "--mode", choices=["tcp", "zmq"], default="zmq", help="Mode: tcp or zmq"
    )
    parser.add_argument(
        "--color",
        choices=["blue", "red"],
        default="blue",
        help="Team color: blue or red",
    )
    parser.add_argument(
        "--rl-checkpoint",
        type=str,
        default=DEFAULT_RL_CHECKPOINT,
        help="Path to the trained PPO kick checkpoint.",
    )
    args = parser.parse_args()

    global our_color, _rl_tree_loaded
    our_color = args.color

    backend = ZMQBackend(our_color) if args.mode == "zmq" else TCPBackend()

    print(f"[Strategy node] {args.mode} mode: started")
    print("[Strategy node] controlling BOTH teams through BT role trees")

    rl_enabled = False
    frame = 0

    while True:
        raw = backend.receive_state()
        if raw is None:
            continue

        for msg in backend.drain_manual():
            if "rl_kick_enabled" in msg:
                want = bool(msg["rl_kick_enabled"])
                if want and not _rl_tree_loaded:
                    _rl_tree_loaded = load_rl_skill(args.rl_checkpoint)
                    if not _rl_tree_loaded and _ckpts:
                        _rl_tree_loaded = load_rl_skill(_ckpts[-1])
                rl_enabled = want and _rl_tree_loaded
                set_rl_enabled(rl_enabled)
                print(f"[Strategy node] RL kick → {'ON' if rl_enabled else 'OFF'}")

        try:
            gamestate = build_game_state(raw, our_color)
            targets, attacker_ids = decide(gamestate)

            backend.send_targets(
                {
                    "targets": {
                        str(rid): {
                            "x": _pos_to_xy(cmd["pos"])[0],
                            "y": _pos_to_xy(cmd["pos"])[1],
                            "angle": cmd["angle"],
                        }
                        for rid, cmd in targets.items()
                    },
                    "attackers": sorted(attacker_ids),
                }
            )

            for rid, cmd in targets.items():
                if cmd.get("kick", False):
                    backend.send_kick(rid)

            for robot in gamestate.blue + gamestate.red:
                cmd = targets.get(robot.id)
                backend.send_dribble(
                    robot.id,
                    bool(cmd is not None and cmd.get("dribble", False)),
                )
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


if __name__ == "__main__":
    main()
