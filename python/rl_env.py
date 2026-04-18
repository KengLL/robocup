"""
Gymnasium-compatible env wrapping SoccerEnv + ObservationWrapper + Reward.

Single-agent, centralized control: one policy outputs commands for all
three blue robots simultaneously. Red runs the same scripted strategy
shipped in the live pipeline (see scripted_opponent.py) so the policy
learns against a moving, ball-chasing opponent — without one, blue
converges to "kick once, then orbit" because nothing punishes
disengagement.

Action layout (Box[-1, 1]^12). Matches the existing pipeline's
strategy_node → robot_node interface: the RL policy picks high-level
*targets* (position + heading + kick intent), and the low-level
dynamic-inversion controller (lifted from robot_node.py) turns those
into wheel commands — same controller, same gains, same physics as
deployment, so the only thing sim-to-real has to transfer is the
strategy, not the motor loop.

    [r0_tx, r0_ty, r0_ttheta, r0_kick,
     r1_tx, r1_ty, r1_ttheta, r1_kick,
     r2_tx, r2_ty, r2_ttheta, r2_kick]

  - tx, ty    ∈ [-1, 1] → target position  ((tx+1)/2 * FIELD_W, same y)
  - ttheta    ∈ [-1, 1] → target heading   (ttheta * π)
  - kick      ∈ [-1, 1] → fires when > 0

Observation layout (Box, flat float32):
    ball   : x, y, vx, vy                                       (4)
    robots : per-robot (x, y, cosθ, sinθ, vx, vy, omega) × 6   (42)
    total  : 46
All distances normalized to roughly [-1, 1] using FIELD_W / FIELD_H.

Episode ends when:
  - a goal is scored by either team   (terminated = True)
  - max_steps reached                  (truncated  = True)
"""

from __future__ import annotations

import math
import random
from typing import Any

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from config import FIELD_H, FIELD_W, NUM_ROBOTS
from observation_wrapper import ObservationWrapper, ObservationWrapperConfig
from rewards import Reward, RewardConfig
from robot_node import RobotController
from scripted_opponent import scripted_red_targets
from soccer_env import SoccerEnv

OBS_DIM = 4 + NUM_ROBOTS * 7
N_OURS = 3
ACT_DIM = N_OURS * 4   # per robot: (target_x, target_y, target_theta, kick)


class RoboCupRLEnv(gym.Env):
    """Single-agent gymnasium env. Controls the blue team (robots 0-2)."""

    metadata = {"render_modes": []}

    def __init__(
        self,
        max_steps: int = 600,                  # 10 s at 60 Hz (physics ticks)
        observation_config: ObservationWrapperConfig | None = None,
        reward_config: RewardConfig | None = None,
        terminate_on_goal: bool = True,
        action_repeat: int = 4,                # policy decides every k physics ticks
        action_smoothness_coef: float = 0.005, # -coef * ||a_t - a_{t-1}||^2 penalty
        controller_mode: str = "2005_INVERSION",
        num_active_opponents: int = 2,         # 3=full strength, 2=no goalie, 1=chaser only
        randomize_ball_spawn: bool = False,    # uniform over field (minus wall margin)
    ) -> None:
        super().__init__()

        self.max_steps = max_steps
        self.terminate_on_goal = terminate_on_goal
        self.action_repeat = max(1, int(action_repeat))
        self.action_smoothness_coef = float(action_smoothness_coef)
        self.controller_mode = controller_mode
        self.num_active_opponents = max(0, min(int(num_active_opponents), N_OURS))
        self.randomize_ball_spawn = bool(randomize_ball_spawn)

        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(ACT_DIM,), dtype=np.float32
        )
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(OBS_DIM,), dtype=np.float32
        )

        self._env = SoccerEnv()
        self._obs_wrapper = ObservationWrapper(observation_config)
        self._reward = Reward(reward_config)

        # The same low-level controller that robot_node runs on real
        # robots. Trained this way the policy learns *targets*, not
        # motor commands — so the motor loop doesn't have to sim-to-real.
        self._controllers = [
            RobotController(i, mode=controller_mode, quiet=True)
            for i in range(N_OURS)
        ]
        # Red team runs the scripted strategy through an identical
        # controller, so red moves exactly like the pipeline's blue team
        # would — realistic sparring partner, zero sim-to-real for the
        # opponent.
        self._red_controllers = [
            RobotController(N_OURS + i, mode=controller_mode, quiet=True)
            for i in range(N_OURS)
        ]

        self._step_count = 0
        self._last_corrupted: dict | None = None
        self._last_action: np.ndarray | None = None

    # ── gym API ─────────────────────────────────────────────────────────

    def reset(
        self, *, seed: int | None = None, options: dict | None = None
    ) -> tuple[np.ndarray, dict]:
        super().reset(seed=seed)
        if seed is not None:
            random.seed(seed)
            np.random.seed(seed)

        true_obs = self._env.reset(seed=seed)
        if self.randomize_ball_spawn:
            # Uniform in-bounds spawn, well inside the walls so the ball
            # isn't already touching anything. np.random is the seeded
            # global state above → reproducible for a given seed.
            bx = float(np.random.uniform(0.7, FIELD_W - 0.7))
            by = float(np.random.uniform(0.7, FIELD_H - 0.7))
            self._env.ball.position = (bx, by)
            self._env.ball.velocity = (0.0, 0.0)
            self._env.ball.angular_velocity = 0.0
            true_obs = self._env.get_obs()
        self._obs_wrapper.reset(seed=seed)
        corrupted = self._obs_wrapper.observe(true_obs)
        # Reward potential needs a starting reference; use the corrupted
        # obs so its potential matches what the policy actually sees.
        self._reward.reset(corrupted)
        for c in list(self._controllers) + list(self._red_controllers):
            c.target = None
            c.target_angle = None
            c._last_pos = None
            c.path_length = 0.0
            c.total_time = 0.0
        self._step_count = 0
        self._last_corrupted = corrupted
        self._last_action = None
        return self._flatten(corrupted), {}

    def step(
        self, action: np.ndarray
    ) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        action = np.clip(np.asarray(action, dtype=np.float32), -1.0, 1.0)
        kicks = self._decode_action(action)

        # Action smoothness penalty: paid once per *policy* decision, not
        # once per physics tick, since the target (and thus the motor
        # loop's command) is held constant across action_repeat ticks.
        if self._last_action is not None:
            smoothness_penalty = self.action_smoothness_coef * float(
                np.sum((action - self._last_action) ** 2)
            )
        else:
            smoothness_penalty = 0.0

        total_reward = -smoothness_penalty
        goal_team: str | None = None
        kicked_total: list[int] = []
        corrupted = self._last_corrupted

        # Update red's scripted targets once per policy decision. That
        # matches the live pipeline cadence (strategy_node publishes at
        # roughly this rate) and avoids resetting the red controllers'
        # path-length bookkeeping every physics tick.
        red_true = self._env.get_obs()
        # Clear stale targets so reds we just dropped from the active set
        # stop moving instead of coasting toward last frame's waypoint.
        for c in self._red_controllers:
            c.target = None
            c.target_angle = None
        for rid, (tx, ty, tth) in scripted_red_targets(
            red_true, num_active=self.num_active_opponents,
        ).items():
            self._red_controllers[rid - N_OURS].set_target(tx, ty, theta=tth)

        for tick in range(self.action_repeat):
            # Controllers run every physics tick using the TRUE robot
            # state (they model the onboard control loop with real
            # encoders / IMU, not the noisy vision pipeline).
            true_now = self._env.get_obs()
            wheels = {
                i: self._controllers[i].compute_wheels(true_now["robots"][str(i)])
                for i in range(N_OURS)
            }
            for i in range(N_OURS):
                rid = N_OURS + i
                wheels[rid] = self._red_controllers[i].compute_wheels(
                    true_now["robots"][str(rid)]
                )
            env_action = {
                "wheels": wheels,
                # Fire kick once per policy decision, not once per physics
                # tick — even though _try_kick_ball is idempotent after
                # the ball leaves, firing repeatedly clutters the log.
                "kicks": kicks if tick == 0 else [],
            }

            true_obs, _, _, info = self._env.step(env_action)
            corrupted = self._obs_wrapper.observe(true_obs)
            total_reward += self._reward.step(corrupted, info)

            if info["goal"] is not None and goal_team is None:
                goal_team = info["goal"]
            kicked_total.extend(info["kicked"])

            self._step_count += 1
            if (self.terminate_on_goal and goal_team is not None) \
                    or self._step_count >= self.max_steps:
                break

        self._last_corrupted = corrupted
        self._last_action = action.copy()

        terminated = self.terminate_on_goal and goal_team is not None
        truncated = self._step_count >= self.max_steps
        info_out = {"goal": goal_team, "kicked": kicked_total}

        return (
            self._flatten(corrupted),
            float(total_reward),
            terminated,
            truncated,
            info_out,
        )

    # ── helpers ─────────────────────────────────────────────────────────

    def _decode_action(self, action: np.ndarray) -> list[int]:
        """Interpret the 12-d action, set targets on the embedded
        controllers, and return the list of robots firing a kick this
        decision."""
        kicks: list[int] = []
        for i in range(N_OURS):
            base = i * 4
            tx_norm = float(action[base + 0])
            ty_norm = float(action[base + 1])
            ttheta_norm = float(action[base + 2])
            kick = float(action[base + 3])

            target_x = (tx_norm + 1.0) / 2.0 * FIELD_W
            target_y = (ty_norm + 1.0) / 2.0 * FIELD_H
            target_theta = ttheta_norm * math.pi
            self._controllers[i].set_target(
                target_x, target_y, theta=target_theta
            )
            if kick > 0.0:
                kicks.append(i)
        return kicks

    def _flatten(self, corrupted: dict) -> np.ndarray:
        out = np.zeros(OBS_DIM, dtype=np.float32)
        b = corrupted["ball"]
        out[0] = (b["x"] - FIELD_W / 2.0) / (FIELD_W / 2.0)
        out[1] = (b["y"] - FIELD_H / 2.0) / (FIELD_H / 2.0)
        out[2] = b["vx"]
        out[3] = b["vy"]

        for rid in range(NUM_ROBOTS):
            r = corrupted["robots"][str(rid)]
            base = 4 + rid * 7
            out[base + 0] = (r["x"] - FIELD_W / 2.0) / (FIELD_W / 2.0)
            out[base + 1] = (r["y"] - FIELD_H / 2.0) / (FIELD_H / 2.0)
            out[base + 2] = math.cos(r["angle"])
            out[base + 3] = math.sin(r["angle"])
            out[base + 4] = r["vx"]
            out[base + 5] = r["vy"]
            out[base + 6] = r.get("omega", 0.0)
        return out
