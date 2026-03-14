#!/usr/bin/env python3
"""
Robot Node — iterative control for each robot.

Subscribes to world state    (VISION_PORT  ZMQ SUB).
Subscribes to manual targets (MANUAL_PORT  ZMQ SUB).
Pushes wheel commands to simulation (COMMAND_PORT ZMQ PUSH).

Implements the three controllers from robot.gd:
  2005_INVERSION    — PD Dynamic Inversion (robot.gd calculate_dynamic_inversion)
  MPC               — Greedy MPC rollout   (robot.gd calculate_mpc_rollout)
  2005_TIME_OPTIMAL — Bang-bang controller  (robot.gd calculate_time_optimal_2005)
"""
import os, sys, math, json
import numpy as np
import zmq

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from config import *


# ── helpers ──────────────────────────────────────────────────────────────────

def _rot2d(v: np.ndarray, angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


def _inverse_kinematics(vx: float, vy: float, w: float) -> list:
    """
    Local body-frame velocity (vx, vy, ω) → normalised wheel speeds.
    Matches robot.gd inverse kinematics block.
    """
    speeds = [
        -math.sin(alpha) * vx + math.cos(alpha) * vy + WHEEL_DISTANCE * w
        for alpha in WHEEL_ANGLES
    ]
    max_s = max(abs(s) for s in speeds)
    if max_s > 1.0:
        speeds = [s / max_s for s in speeds]
    return speeds


# ── controllers ───────────────────────────────────────────────────────────────

def _ctrl_dynamic_inversion(
    pos: np.ndarray, target: np.ndarray,
    velocity: np.ndarray, angle: float,
) -> tuple:
    """
    robot.gd calculate_dynamic_inversion()
    Imposed error dynamics: ë + C·ė + K·e = 0
    """
    error         = target - pos
    desired_accel = error * KP - velocity * KD
    local_accel   = _rot2d(desired_accel, -angle)
    desired_omega = (0.0 - angle) * 10.0
    return local_accel[0], local_accel[1], desired_omega


def _ctrl_mpc_rollout(
    pos: np.ndarray, target: np.ndarray,
    velocity: np.ndarray, angle: float,
) -> tuple:
    """
    robot.gd calculate_mpc_rollout()
    Greedy MPC: evaluate 8 candidate inputs over MPC_HORIZON steps.
    """
    candidates = np.array([
        [1.0, 0.0], [-1.0, 0.0], [0.0, 1.0], [0.0, -1.0],
        [0.7, 0.7], [-0.7, 0.7], [0.7, -0.7], [-0.7, -0.7],
    ])
    best_input  = np.zeros(2)
    lowest_cost = math.inf

    for inp in candidates:
        sim_pos = pos.copy()
        sim_vel = velocity.copy()
        for _ in range(MPC_HORIZON):
            global_force = _rot2d(inp, angle) * MOTOR_MAX_FORCE * 2.0
            sim_accel    = (global_force / ROBOT_MASS) - sim_vel * LINEAR_DAMP
            sim_vel     += sim_accel * MPC_DT
            sim_pos     += sim_vel   * MPC_DT
        cost = float(np.sum((sim_pos - target) ** 2))
        if cost < lowest_cost:
            lowest_cost = cost
            best_input  = inp

    desired_omega = (0.0 - angle) * 10.0
    return best_input[0], best_input[1], desired_omega


def _ctrl_time_optimal(
    pos: np.ndarray, target: np.ndarray,
    velocity: np.ndarray, angle: float,
) -> tuple:
    """
    robot.gd calculate_time_optimal_2005()
    Bang-bang trajectory: accelerate full until braking distance reached.
    """
    to_target = target - pos
    dist      = float(np.linalg.norm(to_target))
    if dist < 1e-6:
        return 0.0, 0.0, 0.0

    direction     = to_target / dist
    a_max         = (MOTOR_MAX_FORCE * 1.5) / ROBOT_MASS
    current_speed = float(np.dot(velocity, direction))
    stopping_dist = (current_speed ** 2) / (2.0 * a_max) if a_max > 0 else 0.0

    if dist > stopping_dist:
        desired_velocity = direction * 500.0                      # high → saturation
    else:
        desired_velocity = direction * math.sqrt(max(0.0, 2.0 * a_max * dist))

    local_v       = _rot2d(desired_velocity, -angle)
    desired_omega = (0.0 - angle) * 5.0
    return local_v[0], local_v[1], desired_omega


_CONTROLLERS = {
    "2005_INVERSION":    _ctrl_dynamic_inversion,
    "MPC":               _ctrl_mpc_rollout,
    "2005_TIME_OPTIMAL": _ctrl_time_optimal,
}


# ── per-robot state ───────────────────────────────────────────────────────────

class RobotController:
    def __init__(self, robot_id: int, mode: str = "2005_INVERSION"):
        self.id          = robot_id
        self.mode        = mode
        self.target: list | None = None
        self.path_length = 0.0
        self.total_time  = 0.0
        self._last_pos: np.ndarray | None = None

    def set_target(self, x: float, y: float, mode: str | None = None) -> None:
        self.target      = [x, y]
        self.path_length = 0.0
        self.total_time  = 0.0
        self._last_pos   = None
        if mode:
            self.mode = mode

    def compute_wheels(self, rstate: dict) -> list:
        """Given robot state dict, return normalised wheel speeds [w0,w1,w2]."""
        pos   = np.array([rstate["x"],  rstate["y"]])
        vel   = np.array([rstate["vx"], rstate["vy"]])
        angle = rstate["angle"]

        if self._last_pos is not None:
            self.path_length += float(np.linalg.norm(pos - self._last_pos))
        self._last_pos = pos.copy()
        self.total_time += DT

        if self.target is None:
            return [0.0, 0.0, 0.0]

        tgt  = np.array(self.target)
        dist = float(np.linalg.norm(pos - tgt))

        if dist < ARRIVAL_THRESH:
            avg = self.path_length / self.total_time if self.total_time > 0 else 0.0
            print(
                f"[Robot {self.id}] Arrived  mode={self.mode}  "
                f"path={self.path_length:.2f}m  time={self.total_time:.2f}s  "
                f"avg_speed={avg:.2f} m/s"
            )
            self.target = None
            return [0.0, 0.0, 0.0]

        ctrl     = _CONTROLLERS[self.mode]
        vx, vy, w = ctrl(pos, tgt, vel, angle)
        return _inverse_kinematics(vx, vy, w)


# ── main loop ─────────────────────────────────────────────────────────────────

def main() -> None:
    robots = [RobotController(i) for i in range(NUM_ROBOTS)]

    ctx = zmq.Context()

    vision_sub = ctx.socket(zmq.SUB)
    vision_sub.connect(f"tcp://localhost:{VISION_PORT}")
    vision_sub.setsockopt_string(zmq.SUBSCRIBE, "")
    vision_sub.setsockopt(zmq.RCVTIMEO, 100)   # wait up to 100 ms

    manual_sub = ctx.socket(zmq.SUB)
    manual_sub.connect(f"tcp://localhost:{MANUAL_PORT}")
    manual_sub.setsockopt_string(zmq.SUBSCRIBE, "")
    manual_sub.setsockopt(zmq.RCVTIMEO, 0)     # non-blocking

    cmd_push = ctx.socket(zmq.PUSH)
    cmd_push.connect(f"tcp://localhost:{COMMAND_PORT}")

    print(f"[RobotNode] vision←:{VISION_PORT}  manual←:{MANUAL_PORT}  cmds→:{COMMAND_PORT}")

    while True:
        # Drain manual targets (non-blocking)
        while True:
            try:
                msg     = manual_sub.recv_string()
                targets = json.loads(msg).get("targets", {})
                for rid_str, info in targets.items():
                    i = int(rid_str)
                    if 0 <= i < NUM_ROBOTS:
                        robots[i].set_target(
                            info["x"], info["y"],
                            info.get("mode", "2005_INVERSION"),
                        )
            except zmq.Again:
                break

        # Block until next vision frame
        try:
            state = json.loads(vision_sub.recv_string())
        except zmq.Again:
            continue

        for robot in robots:
            rid_str = str(robot.id)
            if rid_str not in state.get("robots", {}):
                continue
            wheel_speeds = robot.compute_wheels(state["robots"][rid_str])
            cmd_push.send_string(json.dumps({
                "robot_id":    robot.id,
                "wheel_speeds": wheel_speeds,
            }))


if __name__ == "__main__":
    main()
