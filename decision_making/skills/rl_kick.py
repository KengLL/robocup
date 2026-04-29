"""Load a trained PPO kick policy and run it per-tick against GameState.

Policy was trained blue-attacks-right. For a red attacker, the state is
reflected through x = FIELD_W/2 before inference and the action's
x-component is negated back — the physics is left-right symmetric so
this works without retraining.
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
from stable_baselines3 import PPO

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(os.path.dirname(_HERE))
sys.path.insert(0, os.path.join(_ROOT, "python"))

from config import FIELD_H, FIELD_W  # noqa: E402

from decision_making.rl.env import SUBGOAL_RANGE, make_kick_obs
from decision_making.state import BallState, RobotState


class RLKickSkill:
    def __init__(self, checkpoint_path: str):
        self._model: PPO = PPO.load(checkpoint_path, device="cpu")
        self._ckpt_path: str = checkpoint_path

    @property
    def checkpoint_path(self) -> str:
        return self._ckpt_path

    def target(
        self,
        robot: RobotState,
        ball: BallState,
        attacks_right: bool = True,
    ) -> tuple[float, float]:
        rx = float(robot.pos[0]); ry = float(robot.pos[1])
        ang = float(robot.angle)
        rvx = float(robot.vel[0]); rvy = float(robot.vel[1])
        omega = float(robot.omega)
        bx = float(ball.pos[0]); by = float(ball.pos[1])
        bvx = float(ball.vel[0]); bvy = float(ball.vel[1])

        if attacks_right:
            obs = make_kick_obs(rx, ry, ang, rvx, rvy, omega, bx, by, bvx, bvy)
            action, _ = self._model.predict(obs, deterministic=True)
            tx = float(np.clip(rx + action[0] * SUBGOAL_RANGE, 0.0, FIELD_W))
            ty = float(np.clip(ry + action[1] * SUBGOAL_RANGE, 0.0, FIELD_H))
            return tx, ty

        # Left-right reflection through x = FIELD_W/2.
        obs = make_kick_obs(
            FIELD_W - rx, ry, math.pi - ang,
            -rvx, rvy, -omega,
            FIELD_W - bx, by, -bvx, bvy,
        )
        action, _ = self._model.predict(obs, deterministic=True)
        tx = float(np.clip(rx + (-action[0]) * SUBGOAL_RANGE, 0.0, FIELD_W))
        ty = float(np.clip(ry + action[1] * SUBGOAL_RANGE, 0.0, FIELD_H))
        return tx, ty
