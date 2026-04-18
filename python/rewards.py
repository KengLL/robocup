"""
Reward shaping for the RoboCup PPO task.

Sparse goal reward alone is the right north star but is far too sparse
for a from-scratch policy to ever stumble onto. We add three shaping
terms — all *potential-based* (the per-step delta of a function of the
state), so an optimal policy under shaping is also optimal under the
sparse goal reward (Ng, Harada & Russell 1999).

Terms (all positive numbers below "we get reward when ..."):

  1. goal               +1.0  on our goal, −1.0 on opponent goal
  2. ball→opp-goal      reward when the ball moves closer to the
                        opponent goal (potential = −distance)
  3. our-closest→ball   reward when the closest blue robot moves
                        closer to the ball
  4. kick-on-target     small bonus when a kick connects AND the ball's
                        post-kick velocity points at the opponent goal
                        (cone of ±20°). Encourages "shoot at goal", not
                        "kick in any direction".

There is intentionally NO penalty for time, collisions, or being far
from the home goal — those are easy to over-tune and lead to
defensive-only behavior. Add them later if the policy converges to a
degenerate strategy.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from config import FIELD_H, FIELD_W


@dataclass
class RewardConfig:
    goal_scale: float = 1.0
    ball_to_goal_scale: float = 0.5         # per (meter closer per tick)
    ball_to_our_goal_scale: float = 0.3     # per (meter farther from our goal)
    robot_to_ball_scale: float = 0.1
    kick_on_target_scale: float = 0.2
    kick_cone_half_angle: float = math.radians(20.0)
    our_color: str = "blue"


class Reward:
    """
    Stateful — needs to remember last tick's distances to compute the
    potential delta. Call ``reset(obs)`` once per episode, then ``step(
    obs, info)`` after every env step.
    """

    def __init__(self, config: RewardConfig | None = None) -> None:
        self.cfg = config or RewardConfig()
        self._last_ball_to_goal: float | None = None
        self._last_ball_to_our_goal: float | None = None
        self._last_closest_to_ball: float | None = None

    # ── target geometry ─────────────────────────────────────────────────

    def _opp_goal(self) -> tuple[float, float]:
        # Blue defends x=0, attacks x=FIELD_W. Red is mirrored.
        gx = FIELD_W if self.cfg.our_color == "blue" else 0.0
        return gx, FIELD_H / 2.0

    def _our_goal(self) -> tuple[float, float]:
        gx = 0.0 if self.cfg.our_color == "blue" else FIELD_W
        return gx, FIELD_H / 2.0

    def _our_robot_ids(self, obs: dict) -> list[int]:
        # First half of the robot dict is blue (0..N/2-1), second half red.
        ids = sorted(int(k) for k in obs["robots"].keys())
        half = len(ids) // 2
        return ids[:half] if self.cfg.our_color == "blue" else ids[half:]

    # ── lifecycle ───────────────────────────────────────────────────────

    def reset(self, obs: dict) -> None:
        gx, gy = self._opp_goal()
        bx, by = obs["ball"]["x"], obs["ball"]["y"]
        self._last_ball_to_goal = math.hypot(gx - bx, gy - by)
        ogx, ogy = self._our_goal()
        self._last_ball_to_our_goal = math.hypot(ogx - bx, ogy - by)
        self._last_closest_to_ball = self._closest_robot_dist(obs)

    def step(self, obs: dict, info: dict) -> float:
        if self._last_ball_to_goal is None:
            raise RuntimeError("Reward.step() called before reset()")

        r = 0.0

        # 1. Sparse goal. NOTE: simulation_node._scoring_team returns
        # the CONCEDING team — the color label of the goal mouth the
        # ball entered, not the team that put it there. So
        # info["goal"] == our_color means we got scored on.
        conceded_team = info.get("goal")
        if conceded_team == self.cfg.our_color:
            r -= self.cfg.goal_scale
        elif conceded_team is not None:
            r += self.cfg.goal_scale

        # 2. Ball-to-opp-goal potential.
        gx, gy = self._opp_goal()
        bx, by = obs["ball"]["x"], obs["ball"]["y"]
        d_ball = math.hypot(gx - bx, gy - by)
        r += self.cfg.ball_to_goal_scale * (self._last_ball_to_goal - d_ball)
        self._last_ball_to_goal = d_ball

        # 2b. Ball-AWAY-from-our-goal potential (defensive). Telescopes
        # with the offensive term across most of the field but adds
        # extra signal when the ball is near our own goal.
        ogx, ogy = self._our_goal()
        d_our = math.hypot(ogx - bx, ogy - by)
        r += self.cfg.ball_to_our_goal_scale * (
            d_our - self._last_ball_to_our_goal
        )
        self._last_ball_to_our_goal = d_our

        # 3. Closest-robot-to-ball potential.
        d_closest = self._closest_robot_dist(obs)
        r += self.cfg.robot_to_ball_scale * (
            self._last_closest_to_ball - d_closest
        )
        self._last_closest_to_ball = d_closest

        # 4. Kick on target.
        for rid in info.get("kicked", []):
            # Use ball velocity *after* the impulse — info["kicked"] is
            # populated post-step, so obs["ball"]["vx/vy"] is correct.
            vx, vy = obs["ball"]["vx"], obs["ball"]["vy"]
            speed = math.hypot(vx, vy)
            if speed < 1e-3:
                continue
            ang = math.atan2(vy, vx)
            target_ang = math.atan2(gy - by, gx - bx)
            # smallest signed angle between ang and target_ang
            diff = (ang - target_ang + math.pi) % (2 * math.pi) - math.pi
            if abs(diff) <= self.cfg.kick_cone_half_angle:
                r += self.cfg.kick_on_target_scale

        return r

    # ── helpers ─────────────────────────────────────────────────────────

    def _closest_robot_dist(self, obs: dict) -> float:
        bx, by = obs["ball"]["x"], obs["ball"]["y"]
        ours = self._our_robot_ids(obs)
        return min(
            math.hypot(obs["robots"][str(i)]["x"] - bx,
                       obs["robots"][str(i)]["y"] - by)
            for i in ours
        )
