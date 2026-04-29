"""Reward terms for the kick skill — kept separate so shaping is easy to tune."""

from __future__ import annotations

from dataclasses import dataclass

# Per-step terms sit around ±1; terminal bonuses dominate.
TIME_PENALTY = -0.01
APPROACH_WEIGHT = 0.5  # * meters of robot-to-ball distance closed
BALL_PROGRESS_WEIGHT = 2.0  # * meters of ball-to-goal distance closed
KICK_VELOCITY_WEIGHT = 1.0  # * ball vx gained toward goal on kick tick
FIRST_KICK_ZONE_BONUS = 5.0  # one-shot on first zone entry

GOAL_SCORED_BONUS = 500.0
OWN_GOAL_PENALTY = -100.0
OUT_OF_FIELD_PENALTY = -100.0


@dataclass
class RewardTerms:
    time: float = 0.0
    approach: float = 0.0
    ball_progress: float = 0.0
    kick_velocity: float = 0.0
    kick_zone_bonus: float = 0.0
    terminal: float = 0.0

    def total(self) -> float:
        return (
            self.time
            + self.approach
            + self.ball_progress
            + self.kick_velocity
            + self.kick_zone_bonus
            + self.terminal
        )


def compute_step_reward(
    prev_dist_to_ball: float,
    curr_dist_to_ball: float,
    prev_ball_to_goal: float,
    curr_ball_to_goal: float,
    ball_vel_toward_goal_on_kick: float,
    first_kick_zone_this_ep: bool,
) -> RewardTerms:
    return RewardTerms(
        time=TIME_PENALTY,
        approach=APPROACH_WEIGHT * (prev_dist_to_ball - curr_dist_to_ball),
        ball_progress=BALL_PROGRESS_WEIGHT * (prev_ball_to_goal - curr_ball_to_goal),
        kick_velocity=KICK_VELOCITY_WEIGHT * max(0.0, ball_vel_toward_goal_on_kick),
        kick_zone_bonus=FIRST_KICK_ZONE_BONUS if first_kick_zone_this_ep else 0.0,
    )
