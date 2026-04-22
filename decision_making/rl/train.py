"""PPO trainer for the kick skill.

python decision_making/rl/train.py --check-only # 10k smoke, 1 env
python decision_making/rl/train.py --timesteps 500000 # real run, 8 envs
"""

from __future__ import annotations

import argparse
import os
import sys
from collections.abc import Callable
from datetime import datetime

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(os.path.dirname(_HERE))
sys.path.insert(0, _ROOT)
sys.path.insert(0, os.path.join(_ROOT, "python"))

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
from stable_baselines3 import PPO  # noqa: E402
from stable_baselines3.common.callbacks import CheckpointCallback  # noqa: E402
from stable_baselines3.common.monitor import Monitor  # noqa: E402
from stable_baselines3.common.vec_env import (  # noqa: E402
    DummyVecEnv,
    SubprocVecEnv,
    VecEnv,
)

from decision_making.rl.env import KickEnv  # noqa: E402

CHECKPOINT_DIR = os.path.join(_HERE, "checkpoints")
TB_LOG_DIR = os.path.join(_HERE, "tb_logs")


def _make_env(seed: int) -> Callable[[], gym.Env[np.ndarray, np.ndarray]]:
    def _init() -> gym.Env[np.ndarray, np.ndarray]:
        return Monitor(KickEnv(seed=seed))

    return _init


def _build_vec_env(n_envs: int, base_seed: int, use_subproc: bool) -> VecEnv:
    factories = [_make_env(base_seed + i) for i in range(n_envs)]
    if use_subproc and n_envs > 1:
        return SubprocVecEnv(factories)
    return DummyVecEnv(factories)


def main() -> None:
    parser = argparse.ArgumentParser()
    _ = parser.add_argument("--timesteps", type=int, default=500_000)
    _ = parser.add_argument("--n-envs", type=int, default=8)
    _ = parser.add_argument("--seed", type=int, default=0)
    _ = parser.add_argument("--check-only", action="store_true")
    _ = parser.add_argument("--run-name", type=str, default=None)
    _ = parser.add_argument(
        "--resume",
        type=str,
        default=None,
        help="Path to a .zip checkpoint. Continues training from it for --timesteps more steps.",
    )
    _ = parser.add_argument(
        "--device",
        type=str,
        default="auto",
        help="Torch device for the policy. 'auto', 'cpu', or 'cuda'. "
        + "GPU rarely helps here: net is tiny (13→64→64→3), batches are 8 envs, "
        + "and envs run on CPU — transfer overhead can dominate.",
    )
    args = parser.parse_args()

    if args.check_only:
        timesteps, n_envs = 10_000, 1
    else:
        timesteps, n_envs = args.timesteps, args.n_envs

    run_name = args.run_name or datetime.now().strftime("kick_%Y%m%d_%H%M%S")
    ckpt_dir = os.path.join(CHECKPOINT_DIR, run_name)
    tb_dir = os.path.join(TB_LOG_DIR, run_name)
    os.makedirs(ckpt_dir, exist_ok=True)
    os.makedirs(tb_dir, exist_ok=True)

    print(
        f"[train] run={run_name}  timesteps={timesteps:,}  n_envs={n_envs}  "
        + f"seed={args.seed}"
    )
    print(f"[train] checkpoints → {ckpt_dir}")
    print(f"[train] tb logs     → {tb_dir}")

    vec_env = _build_vec_env(n_envs, args.seed, use_subproc=not args.check_only)

    ent_coef = 0.003

    if args.resume:
        print(f"[train] resuming from {args.resume}")
        model = PPO.load(
            args.resume,
            env=vec_env,
            tensorboard_log=tb_dir,
            verbose=1,
            device=args.device,
        )
        # PPO.load restores hyperparams from the zip; overwrite so the new
        # ent_coef value in this file actually takes effect.
        model.ent_coef = ent_coef
    else:
        model = PPO(
            "MlpPolicy",
            vec_env,
            n_steps=1024,
            batch_size=256,
            learning_rate=3e-4,
            gamma=0.99,
            gae_lambda=0.95,
            ent_coef=ent_coef,
            policy_kwargs={"net_arch": [64, 64]},
            tensorboard_log=tb_dir,
            seed=args.seed,
            verbose=1,
            device=args.device,
        )

    # CheckpointCallback save_freq is per-env. Turn "every N total steps"
    # into "every N/n_envs env steps".
    save_freq_total = max(timesteps // 10, 1) if args.check_only else 50_000
    ckpt_cb = CheckpointCallback(
        save_freq=max(save_freq_total // n_envs, 1),
        save_path=ckpt_dir,
        name_prefix="ppo_kick",
    )

    try:
        _ = model.learn(
            total_timesteps=timesteps,
            callback=ckpt_cb,
            progress_bar=False,
            reset_num_timesteps=not args.resume,
        )
    finally:
        final_path = os.path.join(ckpt_dir, "final.zip")
        model.save(final_path)
        print(f"[train] final policy saved → {final_path}")
        vec_env.close()


if __name__ == "__main__":
    main()
