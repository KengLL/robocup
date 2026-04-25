# receives world state, runs strategy & control the game
from __future__ import annotations

import argparse
import json
import math
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
    GOAL_Y_MAX,
    GOAL_Y_MIN,
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

    def send_dribble(self, robot_id: int, active: bool) -> None:
        _ = self.cmd_push.send_string(
            json.dumps({"type": "dribble", "robot_id": int(robot_id), "active": bool(active)})
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

    def send_dribble(self, robot_id: int, active: bool) -> None:
        pass  # TCP mode has no command channel; dribble only works in ZMQ mode.


def decide(
    gamestate: GameState,
    rl_skill: "RLKickSkill | None" = None,
    rl_enabled: bool = False,
    sticky_attackers: dict[str, int] | None = None,
    kicked_off: bool = True,
) -> tuple[dict[int, tuple[float, float, float]], list[int], set[int]]:
    # Placeholder for Decision Tree
    targets: dict[int, tuple[float, float, float]] = {}
    kicks: list[int] = []
    attacker_rids: set[int] = set()
    sticky = sticky_attackers if sticky_attackers is not None else {}
    ball = gamestate.ball

    # Goalie ids 0 (blue) and 3 (red) are excluded from strategy: they sit at
    # spawn so the four forwards can play 2v2 without a goalie wandering out.
    STATIC_GOALIE_IDS = {0, 3}

    for team_color, team in (("blue", gamestate.blue), ("red", gamestate.red)):
        attacks_right = team_color == "blue"
        team_dir = 0.0 if attacks_right else math.pi
        active = [r for r in team if r.id not in STATIC_GOALIE_IDS]
        sorted_team = sorted(active, key=lambda r: distance(r.pos, ball.pos))

        # Single-robot teams act as the lone goalie: stay on the line from
        # own goal to the ball at a small standoff. No kick is emitted.
        if len(sorted_team) == 1:
            robot = sorted_team[0]
            tx, ty, ta = _goalie_target(ball.pos, team_color)
            targets[robot.id] = (tx, ty, ta)
            sticky.pop(team_color, None)
            continue

        # Sticky attacker with hysteresis. The supporter's path runs past the
        # ball, so per-tick "closest is attacker" flips the role mid-stride and
        # the wrong robot ends up emitting the kick. Keep the incumbent unless
        # a teammate is meaningfully closer.
        last_rid = sticky.get(team_color)
        incumbent = (
            next((r for r in sorted_team if r.id == last_rid), None)
            if last_rid is not None
            else None
        )
        if incumbent is not None and (
            distance(sorted_team[0].pos, ball.pos) + ATTACKER_HYSTERESIS
            >= distance(incumbent.pos, ball.pos)
        ):
            sorted_team = [incumbent] + [r for r in sorted_team if r.id != incumbent.id]
        sticky[team_color] = sorted_team[0].id

        for i, robot in enumerate(sorted_team):
            if i == 0:  # attacker
                attacker_rids.add(robot.id)
                # Policy was trained with robot west of ball (blue-attacks-right).
                # Mirror works cleanly only when the attacker is on its own side
                # of the ball along the attack axis; otherwise the policy drifts
                # OOD and picks the long way round. Fall back to classic there.
                if attacks_right:
                    rl_in_dist = robot.pos[0] < ball.pos[0] - 0.1
                else:
                    rl_in_dist = robot.pos[0] > ball.pos[0] + 0.1

                if rl_enabled and rl_skill is not None and rl_in_dist:
                    # RL: lock to team_dir. Policy was trained with the robot
                    # at that angle and its action frame assumes it.
                    tx, ty = rl_skill.target(robot, ball, attacks_right=attacks_right)
                    targets[robot.id] = (tx, ty, team_dir)
                else:
                    # Classic: face the motion direction while navigating so the
                    # robot doesn't "shift" sideways with its back to the target.
                    # Once near the approach point, rotate to team_dir to line
                    # up the kick.
                    pt = _approach_behind_ball(ball.pos, opp_goal(team_color))
                    dx, dy = float(pt[0] - robot.pos[0]), float(pt[1] - robot.pos[1])
                    dist = math.hypot(dx, dy)
                    ta = team_dir if dist < ATTACKER_ALIGN_DIST else math.atan2(dy, dx)
                    targets[robot.id] = (float(pt[0]), float(pt[1]), ta)
                # Possession filter below drops duplicates in the same tick.
                kicks.append(robot.id)
            elif i == 1:  # supporter — sits past the ball toward opp goal
                goal = opp_goal(team_color)
                to_goal = (goal - ball.pos) / (np.linalg.norm(goal - ball.pos) + 1e-6)
                # Fixed +y perp so blue/red mirror across midfield instead of
                # point-mirroring through the ball (which sent red diagonally).
                perp = np.array([0.0, 1.0])
                if not kicked_off:
                    # Pre-kickoff: hold on own side so the two supporters don't
                    # cross paths and collide while the attackers set up.
                    pt = ball.pos - to_goal * 1.5 + perp * 1.0
                else:
                    pt = ball.pos + to_goal * 1.5 + perp * 1.0
                targets[robot.id] = (
                    float(pt[0]), float(pt[1]),
                    _face_point(robot.pos, ball.pos, team_dir),  # face the ball, not the target
                )
            else:  # defender — hangs back toward own goal
                goal = our_goal(team_color)
                pt = lerp(goal, ball.pos, 0.3)
                targets[robot.id] = (
                    float(pt[0]), float(pt[1]),
                    _face_point(robot.pos, ball.pos, team_dir),
                )

    # Possession: only the attacker strictly closer to the ball emits a kick.
    # Two opposing kicks in the same tick produce equal-opposite impulses
    # that cancel, leaving the ball pinned between both robots.
    if len(kicks) > 1:
        rid_to_bot = {r.id: r for r in gamestate.blue + gamestate.red}
        closest = min(kicks, key=lambda rid: distance(rid_to_bot[rid].pos, ball.pos))
        kicks = [closest]

    return targets, kicks, attacker_rids


def _ball_in_opp_half_for(team_color: str, ball_x: float) -> bool:
    # Blue attacks +x (right half); red attacks -x (left half).
    if team_color == "blue":
        return ball_x > FIELD_LENGTH / 2.0
    return ball_x < FIELD_LENGTH / 2.0


def _face_point(
    pos: np.ndarray, toward: np.ndarray, fallback: float
) -> float:
    dx, dy = float(toward[0] - pos[0]), float(toward[1] - pos[1])
    if math.hypot(dx, dy) < 0.05:
        return fallback
    return math.atan2(dy, dx)


# Offset, in meters, placed on our side of the ball so body pushes forward.
APPROACH_OFFSET = 0.3
# Per-team lateral offset breaks the symmetry when both attackers contest the ball.
APPROACH_LATERAL = 0.2
# Within this distance of the approach point, the attacker rotates to the kick
# direction. Farther away, it faces the motion direction for natural driving.
ATTACKER_ALIGN_DIST = 0.6
# Incumbent attacker keeps the role unless a teammate is at least this much
# closer to the ball. Bigger than typical per-tick noise and the supporter's
# transient pass-by distance.
ATTACKER_HYSTERESIS = 0.4


# Goalie sits this far in front of the goal line, on the line from own
# goal center to the ball. Small enough that the goalie covers the mouth.
GOALIE_STANDOFF = 0.4
# Keep the goalie a hair inside the posts so it never wedges against a wall.
GOALIE_Y_MARGIN = 0.05


def _goalie_target(
    ball_pos: np.ndarray, team_color: str
) -> tuple[float, float, float]:
    goal = our_goal(team_color)
    to_ball = ball_pos - goal
    n = float(np.linalg.norm(to_ball))
    if n < 1e-6:
        tx, ty = float(goal[0]), float(goal[1])
    else:
        pt = goal + (to_ball / n) * GOALIE_STANDOFF
        tx, ty = float(pt[0]), float(pt[1])

    ty = float(np.clip(ty, GOAL_Y_MIN + GOALIE_Y_MARGIN, GOAL_Y_MAX - GOALIE_Y_MARGIN))

    dx, dy = float(ball_pos[0] - tx), float(ball_pos[1] - ty)
    angle = math.atan2(dy, dx) if math.hypot(dx, dy) > 0.05 else (
        0.0 if team_color == "blue" else math.pi
    )
    return tx, ty, angle


def _approach_behind_ball(
    ball_pos: np.ndarray, opp_goal_pos: np.ndarray
) -> np.ndarray:
    to_goal = opp_goal_pos - ball_pos
    n = np.linalg.norm(to_goal)
    if n < 1e-6:
        return ball_pos
    dir_goal = to_goal / n
    # Both attackers approach from the south so the kickoff layout is mirror-
    # symmetric across midfield. Still breaks the standoff because the two
    # attackers end up offset in x.
    return ball_pos - dir_goal * APPROACH_OFFSET + np.array([0.0, -APPROACH_LATERAL])


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
    print(f"[Strategy node] controlling BOTH teams (blue attacks right, red attacks left)")

    rl_skill = None
    rl_enabled = False
    dribble_attacker_enabled = False
    sticky_attackers: dict[str, int] = {}
    # Flips True once the ball has been kicked/pushed. Before that, supporters
    # hold on their own half so they don't barrel into each other at midfield.
    kicked_off = False
    KICKOFF_BALL_SPEED = 0.1

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
            if "dribble_attacker" in msg:
                dribble_attacker_enabled = bool(msg["dribble_attacker"])
                print(f"[Strategy node] Dribble (attacker) → {'ON' if dribble_attacker_enabled else 'OFF'}")

        try:
            gamestate = build_game_state(raw, our_color)
            if not kicked_off and gamestate.ball.speed > KICKOFF_BALL_SPEED:
                kicked_off = True
                print("[Strategy node] kickoff")
            targets, kicks, attacker_rids = decide(
                gamestate, rl_skill, rl_enabled, sticky_attackers, kicked_off
            )
            backend.send_targets({
                "targets": {
                    str(rid): {"x": tx, "y": ty, "angle": ta}
                    for rid, (tx, ty, ta) in targets.items()
                },
                "attackers": sorted(attacker_rids),
            })
            for rid in kicks:
                backend.send_kick(rid)

            # Route dribble to our team's attacker (closest to ball); everyone
            # else gets OFF so stale state never leaks between attacker changes.
            our_team = gamestate.blue if our_color == "blue" else gamestate.red
            if our_team:
                attacker_rid = min(our_team, key=lambda r: distance(r.pos, gamestate.ball.pos)).id
                for r in our_team:
                    backend.send_dribble(r.id, dribble_attacker_enabled and r.id == attacker_rid)
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
