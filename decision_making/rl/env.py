"""Gymnasium env for the approach-and-kick skill (robot 0 only).

Blue attacker drives at a static ball in the red half and tries to score.
Other robots are parked off-field.
"""

from __future__ import annotations

import math
import os
import sys
from typing import Any, override

import gymnasium as gym
import numpy as np
from gymnasium import spaces

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(os.path.dirname(_HERE))
sys.path.insert(0, os.path.join(_ROOT, "python"))

from config import (  # noqa: E402
    FIELD_H,
    FIELD_W,
    GOAL_Y_MAX,
    GOAL_Y_MIN,
    KD,
    KP,
    ROBOT_RADIUS,
    WHEEL_ANGLES,
    WHEEL_DISTANCE,
)
from pymunk_world import PymunkWorld  # noqa: E402

from decision_making.rl.reward import (  # noqa: E402
    GOAL_SCORED_BONUS,
    OUT_OF_FIELD_PENALTY,
    OWN_GOAL_PENALTY,
    RewardTerms,
    compute_step_reward,
)
from decision_making.skills.kick import (  # noqa: E402
    KICK_ZONE_DEPTH,
    KICK_ZONE_HALF_WIDTH,
)

SUBGOAL_RANGE = 1.5  # meters from robot — action a[0], a[1] are scaled by this
CONTROL_SUBSTEPS = 4  # physics ticks per env step -> 15 Hz policy at DT=1/60
MAX_EPISODE_STEPS = 150  # 150 * 4 / 60 = 10 s

# Reset boundaries (keep a margin from walls).
OWN_HALF_X = (0.5, 4.0)
OPP_HALF_X = (4.5, 8.0)
FIELD_Y = (0.8, 5.2)
MIN_RESET_SEPARATION = 0.5  # meters, robot-to-ball


# ── helpers ──────────────────────────────────────────────────────────────────


def _rot2d(v: np.ndarray, angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


def _inverse_kinematics(vx: float, vy: float, w: float) -> list[float]:
    speeds = [
        -math.sin(alpha) * vx + math.cos(alpha) * vy + WHEEL_DISTANCE * w
        for alpha in WHEEL_ANGLES
    ]
    max_s = max(abs(s) for s in speeds)
    if max_s > 1.0:
        speeds = [s / max_s for s in speeds]
    return speeds


def _pd_wheels(
    pos: np.ndarray,
    target: np.ndarray,
    velocity: np.ndarray,
    angle: float,
) -> list[float]:
    # Copy of robot_node._ctrl_dynamic_inversion — hard-coded to aim at angle=0
    # which is +x = red goal. Fine for blue-attacks-right.
    error = target - pos
    desired_accel = error * KP - velocity * KD
    local_accel = _rot2d(desired_accel, -angle)
    desired_omega = (0.0 - angle) * 10.0
    return _inverse_kinematics(local_accel[0], local_accel[1], desired_omega)


def _ball_in_kick_zone(
    robot_x: float,
    robot_y: float,
    robot_angle: float,
    ball_x: float,
    ball_y: float,
) -> bool:
    # Keep in sync with decision_making.skills.kick.try_kick_ball.
    dx, dy = ball_x - robot_x, ball_y - robot_y
    local_x = math.cos(robot_angle) * dx + math.sin(robot_angle) * dy
    local_y = -math.sin(robot_angle) * dx + math.cos(robot_angle) * dy
    return (
        ROBOT_RADIUS <= local_x <= ROBOT_RADIUS + KICK_ZONE_DEPTH
        and abs(local_y) <= KICK_ZONE_HALF_WIDTH
    )


GOAL_CENTER: np.ndarray = np.array([FIELD_W, (GOAL_Y_MIN + GOAL_Y_MAX) / 2.0])


def make_kick_obs(
    rx: float, ry: float, angle: float,
    rvx: float, rvy: float, omega: float,
    bx: float, by: float, bvx: float, bvy: float,
) -> np.ndarray:
    """Build the 13-dim KickEnv obs. Shared by the env and the deploy skill."""
    ball_world = np.array([bx - rx, by - ry])
    ball_vel_world = np.array([bvx, bvy])
    goal_world = GOAL_CENTER - np.array([rx, ry])
    robot_vel_world = np.array([rvx, rvy])

    ball_local = _rot2d(ball_world, -angle)
    ball_vel_local = _rot2d(ball_vel_world, -angle)
    goal_local = _rot2d(goal_world, -angle)
    robot_vel_local = _rot2d(robot_vel_world, -angle)

    dist = float(np.linalg.norm(ball_world))
    in_zone = _ball_in_kick_zone(rx, ry, angle, bx, by)

    return np.array(
        [
            ball_local[0], ball_local[1],
            ball_vel_local[0], ball_vel_local[1],
            goal_local[0], goal_local[1],
            robot_vel_local[0], robot_vel_local[1],
            math.sin(angle), math.cos(angle),
            omega,
            dist,
            1.0 if in_zone else 0.0,
        ],
        dtype=np.float32,
    )


def decode_action_to_target(
    rx: float, ry: float, action: np.ndarray
) -> tuple[float, float]:
    """Decode a policy action (3-dim) into a clipped world-frame target."""
    tx = float(np.clip(rx + action[0] * SUBGOAL_RANGE, 0.0, FIELD_W))
    ty = float(np.clip(ry + action[1] * SUBGOAL_RANGE, 0.0, FIELD_H))
    return tx, ty


# ── env ──────────────────────────────────────────────────────────────────────


class KickEnv(gym.Env[np.ndarray, np.ndarray]):
    metadata: dict[str, Any] = {"render_modes": []}

    OBS_LAYOUT: tuple[str, ...] = (
        "ball_dx",
        "ball_dy",  # ball pos in robot frame, meters
        "ball_vx_local",
        "ball_vy_local",  # ball vel in robot frame, m/s
        "goal_dx",
        "goal_dy",  # opp goal center in robot frame, meters
        "robot_vx_local",
        "robot_vy_local",
        "sin_angle",
        "cos_angle",
        "omega",
        "dist_to_ball",
        "in_kick_zone",  # 0 / 1
    )
    OBS_DIM: int = len(OBS_LAYOUT)

    GOAL_CENTER: np.ndarray = GOAL_CENTER

    def __init__(self, seed: int | None = None):
        super().__init__()

        self.action_space: spaces.Space[np.ndarray] = spaces.Box(
            low=-1.0, high=1.0, shape=(3,), dtype=np.float32
        )

        # Positional features live in the robot's rotated frame, so their
        # magnitude can reach the field diagonal (~10.8 m), not axis length.
        diag = math.hypot(FIELD_W, FIELD_H) + 1.0
        obs_high = np.array(
            [
                diag,
                diag,  # ball pos in robot frame
                15.0,
                15.0,  # ball vel in robot frame — single kick adds ~5 m/s; slack for bounces
                diag,
                diag,  # opp goal pos in robot frame
                6.0,
                6.0,  # robot vel in own frame
                1.0,
                1.0,  # sin / cos of robot angle
                20.0,  # omega
                diag,  # distance to ball
                1.0,  # in kick zone flag
            ],
            dtype=np.float32,
        )
        obs_low = -obs_high.copy()
        obs_low[11] = 0.0  # distance is non-negative
        obs_low[12] = 0.0  # flag is 0/1
        self.observation_space: spaces.Space[np.ndarray] = spaces.Box(
            low=obs_low, high=obs_high, shape=(self.OBS_DIM,), dtype=np.float32
        )

        self._world: PymunkWorld = PymunkWorld(seed=None)
        self._rng: np.random.Generator = np.random.default_rng(seed)

        self._steps: int = 0
        self._prev_dist_to_ball: float = 0.0
        self._prev_ball_to_goal: float = 0.0
        self._ever_in_kick_zone: bool = False
        self._last_terms: RewardTerms = RewardTerms()

    @override
    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[np.ndarray, dict[str, Any]]:
        _ = super().reset(seed=seed)
        if seed is not None:
            self._rng = np.random.default_rng(seed)

        # Park non-attackers far off-field so they can't collide.
        for rid in range(1, len(self._world.robots)):
            self._world.set_robot(rid, (-10.0 - rid, -10.0), 0.0)

        ball_xy = self._sample_ball_xy()
        self._world.reset_ball((float(ball_xy[0]), float(ball_xy[1])))

        robot_xy = self._sample_robot_xy(ball_xy)
        robot_angle = float(self._rng.uniform(-math.pi, math.pi))
        self._world.set_robot(0, (float(robot_xy[0]), float(robot_xy[1])), robot_angle)

        self._steps = 0
        self._ever_in_kick_zone = False
        self._prev_dist_to_ball = float(np.linalg.norm(robot_xy - ball_xy))
        self._prev_ball_to_goal = float(np.linalg.norm(ball_xy - self.GOAL_CENTER))
        self._last_terms = RewardTerms()

        return self._observe(), self._info(terminal="")

    @override
    def step(
        self, action: np.ndarray
    ) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        action = np.clip(np.asarray(action, dtype=np.float32), -1.0, 1.0)

        robot = self._world.robots[0]
        target = np.array(
            [
                np.clip(robot.position.x + action[0] * SUBGOAL_RANGE, 0.0, FIELD_W),
                np.clip(robot.position.y + action[1] * SUBGOAL_RANGE, 0.0, FIELD_H),
            ]
        )

        ball_vx_toward_goal_on_kick = 0.0
        entered_zone_this_step = False
        scoring_team: str | None = None

        for _ in range(CONTROL_SUBSTEPS):
            pos = np.array([robot.position.x, robot.position.y])
            vel = np.array([robot.velocity.x, robot.velocity.y])
            angle = robot.angle
            wheels = _pd_wheels(pos, target, vel, angle)

            if not self._ever_in_kick_zone and _ball_in_kick_zone(
                pos[0],
                pos[1],
                angle,
                self._world.ball.position.x,
                self._world.ball.position.y,
            ):
                self._ever_in_kick_zone = True
                entered_zone_this_step = True

            # Auto-kick every tick. The zone gate inside try_kick_ball filters.
            vx_before = self._world.ball.velocity.x
            state = self._world.step({0: wheels}, kicks=(0,))
            if state["kicks"]:
                ball_vx_toward_goal_on_kick += self._world.ball.velocity.x - vx_before

            if state["scoring_team"] is not None:
                scoring_team = state["scoring_team"]
                break

        self._steps += 1

        rx, ry = self._world.robots[0].position.x, self._world.robots[0].position.y
        bx, by = self._world.ball.position.x, self._world.ball.position.y
        curr_dist = float(math.hypot(rx - bx, ry - by))
        curr_ball_to_goal = float(
            math.hypot(bx - self.GOAL_CENTER[0], by - self.GOAL_CENTER[1])
        )

        terms = compute_step_reward(
            prev_dist_to_ball=self._prev_dist_to_ball,
            curr_dist_to_ball=curr_dist,
            prev_ball_to_goal=self._prev_ball_to_goal,
            curr_ball_to_goal=curr_ball_to_goal,
            ball_vel_toward_goal_on_kick=ball_vx_toward_goal_on_kick,
            first_kick_zone_this_ep=entered_zone_this_step,
        )
        self._prev_dist_to_ball = curr_dist
        self._prev_ball_to_goal = curr_ball_to_goal

        terminated = False
        terminal_reason = ""
        if scoring_team == "red":
            terms.terminal = GOAL_SCORED_BONUS
            terminated = True
            terminal_reason = "goal"
            self._world.reset_ball()
        elif scoring_team == "blue":
            terms.terminal = OWN_GOAL_PENALTY
            terminated = True
            terminal_reason = "own_goal"
            self._world.reset_ball()
        elif not (0.0 <= rx <= FIELD_W and 0.0 <= ry <= FIELD_H):
            terms.terminal = OUT_OF_FIELD_PENALTY
            terminated = True
            terminal_reason = "robot_out"

        truncated = not terminated and self._steps >= MAX_EPISODE_STEPS

        self._last_terms = terms
        return (
            self._observe(),
            terms.total(),
            terminated,
            truncated,
            self._info(terminal=terminal_reason),
        )

    # ── internals ────────────────────────────────────────────────────────────

    def _observe(self) -> np.ndarray:
        robot = self._world.robots[0]
        ball = self._world.ball
        return make_kick_obs(
            robot.position.x, robot.position.y, robot.angle,
            robot.velocity.x, robot.velocity.y, robot.angular_velocity,
            ball.position.x, ball.position.y, ball.velocity.x, ball.velocity.y,
        )

    def _info(self, terminal: str) -> dict[str, Any]:
        t = self._last_terms
        return {
            "terminal": terminal,
            "reward/time": t.time,
            "reward/approach": t.approach,
            "reward/ball_progress": t.ball_progress,
            "reward/kick_velocity": t.kick_velocity,
            "reward/kick_zone_bonus": t.kick_zone_bonus,
            "reward/terminal": t.terminal,
        }

    def _sample_ball_xy(self) -> np.ndarray:
        return np.array(
            [
                float(self._rng.uniform(*OPP_HALF_X)),
                float(self._rng.uniform(*FIELD_Y)),
            ]
        )

    def _sample_robot_xy(self, ball_xy: np.ndarray) -> np.ndarray:
        for _ in range(64):
            xy = np.array(
                [
                    float(self._rng.uniform(*OWN_HALF_X)),
                    float(self._rng.uniform(*FIELD_Y)),
                ]
            )
            if np.linalg.norm(xy - ball_xy) >= MIN_RESET_SEPARATION:
                return xy
        return np.array(
            [OWN_HALF_X[0], FIELD_Y[0]]
        )  # unreachable unless RNG pathological


# ── smoke test ───────────────────────────────────────────────────────────────

if __name__ == "__main__":  # pragma: no cover
    import argparse

    parser = argparse.ArgumentParser()
    _ = parser.add_argument("--check", action="store_true")
    _ = parser.add_argument("--episodes", type=int, default=5)
    args = parser.parse_args()

    env = KickEnv(seed=0)

    if args.check:
        from gymnasium.utils.env_checker import check_env

        check_env(env, skip_render_check=True)
        print("check_env: OK")

    rng = np.random.default_rng(0)
    rewards: list[float] = []
    for ep in range(args.episodes):
        obs, _ = env.reset(seed=ep)
        ep_r = 0.0
        steps = 0
        while True:
            a = rng.uniform(-1.0, 1.0, size=3).astype(np.float32)
            obs, r, term, trunc, info = env.step(a)
            ep_r += r
            steps += 1
            if term or trunc:
                rewards.append(ep_r)
                print(
                    f"ep={ep}  steps={steps}  reward={ep_r:+.2f}  "
                    + f"term={info.get('terminal', '') or '(truncated)'}"
                )
                break
    print(f"mean random-policy reward: {sum(rewards) / len(rewards):+.2f}")
