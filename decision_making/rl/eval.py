"""Evaluate a trained kick policy on held-out seeds."""

from __future__ import annotations

import argparse
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(os.path.dirname(_HERE))
sys.path.insert(0, _ROOT)
sys.path.insert(0, os.path.join(_ROOT, "python"))

from stable_baselines3 import PPO  # noqa: E402

from decision_making.rl.env import KickEnv  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser()
    _ = parser.add_argument("--checkpoint", type=str, required=True)
    _ = parser.add_argument("--episodes", type=int, default=100)
    # Use a seed range disjoint from training to avoid leakage.
    _ = parser.add_argument("--seed-start", type=int, default=10_000)
    _ = parser.add_argument("--stochastic", action="store_true")
    args = parser.parse_args()

    model = PPO.load(args.checkpoint)
    env = KickEnv(seed=args.seed_start)

    goals = own_goals = truncated = robot_out = 0
    rewards: list[float] = []
    lengths: list[int] = []

    for ep in range(args.episodes):
        obs, _ = env.reset(seed=args.seed_start + ep)
        total = 0.0
        steps = 0
        while True:
            action, _ = model.predict(obs, deterministic=not args.stochastic)
            obs, r, term, trunc, info = env.step(action)
            total += float(r)
            steps += 1
            if term or trunc:
                rewards.append(total)
                lengths.append(steps)
                reason = info.get("terminal", "")
                if reason == "goal":
                    goals += 1
                elif reason == "own_goal":
                    own_goals += 1
                elif reason == "robot_out":
                    robot_out += 1
                else:
                    truncated += 1
                break

    n = args.episodes
    print(f"checkpoint: {args.checkpoint}")
    print(f"episodes:   {n}  (seeds {args.seed_start}..{args.seed_start + n - 1})")
    print(f"mode:       {'stochastic' if args.stochastic else 'deterministic'}")
    print(f"  goals:       {goals:4d}  ({100 * goals / n:5.1f}%)")
    print(f"  own goals:   {own_goals:4d}  ({100 * own_goals / n:5.1f}%)")
    print(f"  robot out:   {robot_out:4d}  ({100 * robot_out / n:5.1f}%)")
    print(f"  truncated:   {truncated:4d}  ({100 * truncated / n:5.1f}%)")
    print(f"  mean reward: {sum(rewards) / n:+.2f}")
    print(f"  mean length: {sum(lengths) / n:.1f} env-steps  (max 150)")


if __name__ == "__main__":
    main()
