"""
Deterministic evaluation for a trained PPO checkpoint.

Runs N episodes with the deterministic mean action (no sampling), no
action smoothness penalty in the reported reward, and logs:

  - mean / std episode return
  - goals scored for and against
  - mean episode length
  - mean action-change magnitude (proxy for motion smoothness)

Use this to confirm coherent behavior before/after retraining.
"""

from __future__ import annotations

import argparse
import math

import numpy as np
import torch

from rewards import RewardConfig
from rl_env import ACT_DIM, OBS_DIM, RoboCupRLEnv
from train_ppo import ActorCritic, RunningMeanStd


def mean_teammate_distance(true_obs: dict) -> float:
    p0 = true_obs["robots"]["0"]
    p1 = true_obs["robots"]["1"]
    p2 = true_obs["robots"]["2"]
    pairs = [
        math.hypot(p0["x"] - p1["x"], p0["y"] - p1["y"]),
        math.hypot(p0["x"] - p2["x"], p0["y"] - p2["y"]),
        math.hypot(p1["x"] - p2["x"], p1["y"] - p2["y"]),
    ]
    return float(np.mean(pairs))


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--checkpoint", required=True)
    p.add_argument("--episodes", type=int, default=20)
    p.add_argument("--action-repeat", type=int, default=4)
    p.add_argument("--max-steps", type=int, default=600)
    p.add_argument("--seed", type=int, default=1000)
    p.add_argument("--deterministic", action="store_true", default=True)
    p.add_argument("--stochastic", action="store_false", dest="deterministic")
    p.add_argument(
        "--num-active-opponents", type=int, default=2,
        help="Must match the training config or scores will be meaningless.",
    )
    p.add_argument(
        "--random-ball-prob", type=float, default=0.5,
        help="Match the training ball-spawn distribution.",
    )
    p.add_argument(
        "--randomize-ball", action="store_true",
        help="Backward-compatible alias for --random-ball-prob 1.0",
    )
    p.add_argument(
        "--teammate-spacing-scale", type=float, default=0.0,
        help="Evaluation reward config parity with training.",
    )
    p.add_argument(
        "--teammate-spacing-threshold", type=float, default=0.9,
        help="Evaluation reward config parity with training.",
    )
    p.add_argument(
        "--reward-source",
        choices=["true", "corrupted"],
        default="true",
        help="State used for reward shaping: true or corrupted.",
    )
    args = p.parse_args()

    if args.randomize_ball:
        args.random_ball_prob = 1.0
    args.random_ball_prob = float(np.clip(args.random_ball_prob, 0.0, 1.0))

    ckpt = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
    model = ActorCritic(OBS_DIM, ACT_DIM)
    model.load_state_dict(ckpt["model_state_dict"])
    model.eval()

    rms = RunningMeanStd((OBS_DIM,))
    rms.mean = np.asarray(ckpt["obs_rms_mean"])
    rms.var = np.asarray(ckpt["obs_rms_var"])
    rms.count = float(ckpt["obs_rms_count"])

    env = RoboCupRLEnv(
        max_steps=args.max_steps,
        action_repeat=args.action_repeat,
        action_smoothness_coef=0.0,  # don't pollute eval reward with smoothness term
        num_active_opponents=args.num_active_opponents,
        random_ball_prob=args.random_ball_prob,
        reward_use_corrupted_obs=(args.reward_source == "corrupted"),
        reward_config=RewardConfig(
            teammate_spacing_scale=args.teammate_spacing_scale,
            teammate_spacing_threshold=args.teammate_spacing_threshold,
        ),
    )

    returns = []
    lengths = []
    goals_for = 0
    goals_against = 0
    action_deltas = []
    teammate_distances = []
    wins = 0
    losses = 0
    draws = 0

    for ep in range(args.episodes):
        obs, _ = env.reset(seed=args.seed + ep)
        ep_return = 0.0
        ep_len = 0
        last_action = None
        while True:
            obs_t = torch.as_tensor(
                rms.normalize(obs[None, :]), dtype=torch.float32
            )
            with torch.no_grad():
                action, _, _ = model.act(obs_t, deterministic=args.deterministic)
            a = action.squeeze(0).numpy()
            if last_action is not None:
                action_deltas.append(float(np.linalg.norm(a - last_action)))
            last_action = a

            obs, r, term, trunc, info = env.step(a)
            ep_return += r
            ep_len += 1
            teammate_distances.append(mean_teammate_distance(env._env.get_obs()))
            # info["goal"] is the conceding team (see rewards.py).
            if info.get("goal") == "red":
                goals_for += 1
            elif info.get("goal") == "blue":
                goals_against += 1
            if term or trunc:
                if info.get("goal") == "red":
                    wins += 1
                elif info.get("goal") == "blue":
                    losses += 1
                else:
                    draws += 1
                break

        returns.append(ep_return)
        lengths.append(ep_len)
        print(
            f"  ep {ep:02d}  ret={ep_return:+.3f}  len={ep_len:3d}  "
            f"goals=+{goals_for}/-{goals_against}"
        )

    print()
    print(f"episodes         : {args.episodes}")
    print(f"mean return      : {np.mean(returns):+.3f} ± {np.std(returns):.3f}")
    print(f"mean length      : {np.mean(lengths):.1f}")
    print(f"goals for        : {goals_for}")
    print(f"goals against    : {goals_against}")
    print(f"goal diff        : {goals_for - goals_against:+d}")
    print(f"win rate         : {wins / args.episodes:.3f}")
    print(f"draw rate        : {draws / args.episodes:.3f}")
    print(f"mean team dist   : {np.mean(teammate_distances):.3f} m")
    print(f"mean |Δaction|   : {np.mean(action_deltas):.4f}  "
          f"(lower ⇒ smoother motion)")


if __name__ == "__main__":
    main()
