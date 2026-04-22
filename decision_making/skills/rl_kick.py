"""Load a trained PPO kick policy and run it per-tick against GameState.

The policy was trained with robot 0 attacking the red goal (+x direction).
Works for any robot ID because the obs is egocentric, but only when the
caller is blue-attacks-right.
"""

from __future__ import annotations

from stable_baselines3 import PPO

from decision_making.rl.env import decode_action_to_target, make_kick_obs
from decision_making.state import BallState, RobotState


class RLKickSkill:
    def __init__(self, checkpoint_path: str):
        self._model: PPO = PPO.load(checkpoint_path, device="cpu")
        self._ckpt_path: str = checkpoint_path

    @property
    def checkpoint_path(self) -> str:
        return self._ckpt_path

    def target(self, robot: RobotState, ball: BallState) -> tuple[float, float]:
        obs = make_kick_obs(
            float(robot.pos[0]), float(robot.pos[1]), float(robot.angle),
            float(robot.vel[0]), float(robot.vel[1]), float(robot.omega),
            float(ball.pos[0]), float(ball.pos[1]),
            float(ball.vel[0]), float(ball.vel[1]),
        )
        action, _ = self._model.predict(obs, deterministic=True)
        return decode_action_to_target(float(robot.pos[0]), float(robot.pos[1]), action)
