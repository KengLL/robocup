"""
Vectorized PPO for RoboCupRLEnv.

Single-file implementation, no SB3 / cleanrl dependency. Designed to
hit 1M+ environment steps comfortably on a laptop CPU.

Adds, vs. the v1 trainer:

  - SubprocVecEnv: N sub-envs in N child processes (true parallelism;
    pymunk + Python GIL no longer a bottleneck).
  - Running observation normalization (Welford), so the network sees
    zero-mean / unit-variance features.
  - Return-based reward normalization (the SB3 / Baselines trick:
    divide rewards by the running std of discounted returns; do NOT
    subtract the mean — that would change the optimal policy).
  - Linear annealing of learning rate and entropy coefficient over the
    full training budget. Less exploration noise late, finer policy
    edits.
  - Lower initial log_std (-1.0 → std ≈ 0.37) so the random initial
    policy doesn't slam wheels around chaotically — combined with
    action_repeat=4 in the env, motion looks smooth from step 1.

Action smoothness is enforced inside the env (rl_env.py applies a
small −coef·||Δa||² penalty per policy decision).

Run::

    python train_ppo.py --total-steps 1_000_000 --n-envs 8 --save out.pt
"""

from __future__ import annotations

import argparse
import os
import time
from dataclasses import dataclass

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions import Normal

from rl_env import ACT_DIM, OBS_DIM, RoboCupRLEnv
from vec_env import SubprocVecEnv


# ── running statistics ─────────────────────────────────────────────────


class RunningMeanStd:
    """Welford's online mean/variance, vector-shaped."""

    def __init__(self, shape: tuple[int, ...]) -> None:
        self.mean = np.zeros(shape, dtype=np.float64)
        self.var = np.ones(shape, dtype=np.float64)
        self.count = 1e-4

    def update(self, x: np.ndarray) -> None:
        batch_mean = x.mean(axis=0)
        batch_var = x.var(axis=0)
        batch_count = x.shape[0]
        delta = batch_mean - self.mean
        tot = self.count + batch_count
        new_mean = self.mean + delta * batch_count / tot
        m_a = self.var * self.count
        m_b = batch_var * batch_count
        m2 = m_a + m_b + delta**2 * self.count * batch_count / tot
        self.mean = new_mean
        self.var = m2 / tot
        self.count = tot

    def normalize(self, x: np.ndarray, clip: float = 10.0) -> np.ndarray:
        return np.clip(
            (x - self.mean) / np.sqrt(self.var + 1e-8),
            -clip,
            clip,
        ).astype(np.float32)


class ReturnNormalizer:
    """Normalize rewards by the running std of discounted returns.
    See OpenAI Baselines VecNormalize."""

    def __init__(self, n_envs: int, gamma: float = 0.99) -> None:
        self.gamma = gamma
        self.returns = np.zeros(n_envs, dtype=np.float64)
        self.rms = RunningMeanStd(())

    def update(self, rewards: np.ndarray, dones: np.ndarray) -> np.ndarray:
        self.returns = self.returns * self.gamma + rewards
        self.rms.update(self.returns)
        scale = np.sqrt(self.rms.var + 1e-8)
        self.returns *= 1.0 - dones.astype(np.float64)
        return rewards / max(scale, 1e-3)


# ── network ────────────────────────────────────────────────────────────


class ActorCritic(nn.Module):
    def __init__(
        self, obs_dim: int, act_dim: int, hidden: int = 128,
        init_log_std: float = -1.0,
    ) -> None:
        super().__init__()
        self.trunk = nn.Sequential(
            nn.Linear(obs_dim, hidden),
            nn.LayerNorm(hidden),
            nn.Tanh(),
            nn.Linear(hidden, hidden),
            nn.LayerNorm(hidden),
            nn.Tanh(),
        )
        self.mean = nn.Linear(hidden, act_dim)
        self.log_std = nn.Parameter(torch.full((act_dim,), float(init_log_std)))
        self.value = nn.Linear(hidden, 1)

        for layer in (self.mean, self.value):
            nn.init.orthogonal_(layer.weight, gain=0.01)
            nn.init.zeros_(layer.bias)

    def forward(self, obs: torch.Tensor):
        h = self.trunk(obs)
        mean = torch.tanh(self.mean(h))
        std = self.log_std.exp().expand_as(mean)
        value = self.value(h).squeeze(-1)
        return mean, std, value

    def act(self, obs: torch.Tensor, deterministic: bool = False):
        mean, std, value = self.forward(obs)
        if deterministic:
            return mean, torch.zeros(mean.shape[0], device=mean.device), value
        dist = Normal(mean, std)
        action = dist.sample()
        logprob = dist.log_prob(action).sum(-1)
        return action, logprob, value

    def evaluate(self, obs: torch.Tensor, action: torch.Tensor):
        mean, std, value = self.forward(obs)
        dist = Normal(mean, std)
        logprob = dist.log_prob(action).sum(-1)
        entropy = dist.entropy().sum(-1)
        return logprob, entropy, value


# ── rollout buffer ─────────────────────────────────────────────────────


@dataclass
class Rollout:
    obs: np.ndarray         # (T, N, obs_dim)
    actions: np.ndarray     # (T, N, act_dim)
    logprobs: np.ndarray    # (T, N)
    rewards: np.ndarray     # (T, N)  — already return-normalized
    dones: np.ndarray       # (T, N)  — terminated OR truncated
    values: np.ndarray      # (T, N)
    advantages: np.ndarray  # (T, N)
    returns: np.ndarray     # (T, N)


def compute_gae(
    rewards: np.ndarray,        # (T, N)
    values: np.ndarray,         # (T, N)
    dones: np.ndarray,          # (T, N)
    last_values: np.ndarray,    # (N,)
    gamma: float,
    lam: float,
) -> tuple[np.ndarray, np.ndarray]:
    T = rewards.shape[0]
    advantages = np.zeros_like(rewards)
    last_adv = np.zeros(rewards.shape[1], dtype=np.float32)
    for t in reversed(range(T)):
        next_values = last_values if t == T - 1 else values[t + 1]
        next_nonterminal = 1.0 - dones[t]
        delta = rewards[t] + gamma * next_values * next_nonterminal - values[t]
        last_adv = delta + gamma * lam * next_nonterminal * last_adv
        advantages[t] = last_adv
    returns = advantages + values
    return advantages, returns


# ── training step pieces ───────────────────────────────────────────────


def make_env_factory(
    seed: int,
    action_repeat: int,
    action_smoothness_coef: float,
    max_steps: int,
    num_active_opponents: int = 2,
    randomize_ball_spawn: bool = False,
    robot_to_ball_scale: float | None = None,
):
    """Returns a no-arg factory that builds one env. Marshalled into a
    child process by SubprocVecEnv via fork."""
    from rewards import RewardConfig
    reward_config = None
    if robot_to_ball_scale is not None:
        reward_config = RewardConfig(robot_to_ball_scale=robot_to_ball_scale)

    def _make() -> RoboCupRLEnv:
        env = RoboCupRLEnv(
            max_steps=max_steps,
            action_repeat=action_repeat,
            action_smoothness_coef=action_smoothness_coef,
            num_active_opponents=num_active_opponents,
            randomize_ball_spawn=randomize_ball_spawn,
            reward_config=reward_config,
        )
        env.reset(seed=seed)
        return env
    return _make


def collect_rollout(
    envs: SubprocVecEnv,
    model: ActorCritic,
    obs: np.ndarray,
    horizon: int,
    obs_rms: RunningMeanStd,
    ret_norm: ReturnNormalizer,
    device: torch.device,
):
    n_envs = envs.n_envs
    obs_buf = np.zeros((horizon, n_envs, OBS_DIM), dtype=np.float32)
    act_buf = np.zeros((horizon, n_envs, ACT_DIM), dtype=np.float32)
    lp_buf = np.zeros((horizon, n_envs), dtype=np.float32)
    rew_buf = np.zeros((horizon, n_envs), dtype=np.float32)
    done_buf = np.zeros((horizon, n_envs), dtype=np.float32)
    val_buf = np.zeros((horizon, n_envs), dtype=np.float32)

    ep_returns: list[float] = []
    ep_lengths: list[int] = []
    goals_for = 0
    goals_against = 0

    for t in range(horizon):
        obs_norm = obs_rms.normalize(obs)
        obs_t = torch.as_tensor(obs_norm, device=device)
        with torch.no_grad():
            action, logprob, value = model.act(obs_t)
        a_np = action.cpu().numpy()

        next_obs, raw_reward, term, trunc, infos = envs.step(a_np)
        done = term | trunc

        # Update running obs stats from RAW (un-normalized) obs.
        obs_rms.update(obs)

        # Reward normalization (in-place per env state).
        norm_reward = ret_norm.update(raw_reward, done.astype(np.float32))

        obs_buf[t] = obs_norm
        act_buf[t] = a_np
        lp_buf[t] = logprob.cpu().numpy()
        rew_buf[t] = norm_reward
        done_buf[t] = done.astype(np.float32)
        val_buf[t] = value.cpu().numpy()

        for i, info in enumerate(infos):
            if "episode" in info:
                ep_returns.append(info["episode"]["return"])
                ep_lengths.append(info["episode"]["length"])
            # info["goal"] is the conceding team (see rewards.py note).
            # Blue is the agent — blue conceding = goal against us.
            if info.get("goal") == "red":
                goals_for += 1
            elif info.get("goal") == "blue":
                goals_against += 1

        obs = next_obs

    with torch.no_grad():
        last_value = model.act(
            torch.as_tensor(obs_rms.normalize(obs), device=device)
        )[2].cpu().numpy()

    advantages, returns = compute_gae(
        rew_buf, val_buf, done_buf, last_value, gamma=0.99, lam=0.95
    )

    rollout = Rollout(
        obs=obs_buf, actions=act_buf, logprobs=lp_buf,
        rewards=rew_buf, dones=done_buf, values=val_buf,
        advantages=advantages, returns=returns,
    )
    stats = {
        "ep_return_mean": float(np.mean(ep_returns)) if ep_returns else 0.0,
        "ep_length_mean": float(np.mean(ep_lengths)) if ep_lengths else 0.0,
        "n_episodes": len(ep_returns),
        "goals_for": goals_for,
        "goals_against": goals_against,
    }
    return rollout, obs, stats


def ppo_update(
    model: ActorCritic,
    optimizer: optim.Optimizer,
    rollout: Rollout,
    *,
    epochs: int,
    batch_size: int,
    clip: float,
    ent_coef: float,
    vf_coef: float,
    target_kl: float,
    device: torch.device,
) -> dict:
    # Flatten (T, N, ...) -> (T*N, ...)
    def flat(x): return x.reshape(-1, *x.shape[2:]) if x.ndim > 2 else x.reshape(-1)
    obs = torch.as_tensor(flat(rollout.obs), device=device)
    actions = torch.as_tensor(flat(rollout.actions), device=device)
    old_lp = torch.as_tensor(flat(rollout.logprobs), device=device)
    advs = torch.as_tensor(flat(rollout.advantages), device=device)
    returns = torch.as_tensor(flat(rollout.returns), device=device)
    old_values = torch.as_tensor(flat(rollout.values), device=device)

    advs = (advs - advs.mean()) / (advs.std() + 1e-8)

    n = obs.shape[0]
    idx = np.arange(n)

    pol_losses, val_losses, ent_means, kl_means, clip_fracs = [], [], [], [], []
    early_stop = False

    for _ in range(epochs):
        np.random.shuffle(idx)
        for start in range(0, n, batch_size):
            b = idx[start:start + batch_size]
            mb_obs = obs[b]
            mb_act = actions[b]
            mb_old_lp = old_lp[b]
            mb_adv = advs[b]
            mb_ret = returns[b]
            mb_old_val = old_values[b]

            new_lp, entropy, value = model.evaluate(mb_obs, mb_act)
            ratio = (new_lp - mb_old_lp).exp()
            unclipped = ratio * mb_adv
            clipped = torch.clamp(ratio, 1.0 - clip, 1.0 + clip) * mb_adv
            policy_loss = -torch.min(unclipped, clipped).mean()

            # Clipped value loss (a la PPO2).
            v_clipped = mb_old_val + torch.clamp(
                value - mb_old_val, -clip, clip
            )
            v_loss_unclipped = (value - mb_ret) ** 2
            v_loss_clipped = (v_clipped - mb_ret) ** 2
            value_loss = 0.5 * torch.max(v_loss_unclipped, v_loss_clipped).mean()

            ent = entropy.mean()
            loss = policy_loss + vf_coef * value_loss - ent_coef * ent

            optimizer.zero_grad()
            loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), 0.5)
            optimizer.step()

            with torch.no_grad():
                approx_kl = (mb_old_lp - new_lp).mean().item()
                clip_fracs.append(
                    ((ratio - 1.0).abs() > clip).float().mean().item()
                )
            pol_losses.append(policy_loss.item())
            val_losses.append(value_loss.item())
            ent_means.append(ent.item())
            kl_means.append(approx_kl)

        if target_kl > 0 and np.mean(kl_means[-(n // batch_size):]) > target_kl:
            early_stop = True
            break

    return {
        "policy_loss": float(np.mean(pol_losses)),
        "value_loss": float(np.mean(val_losses)),
        "entropy": float(np.mean(ent_means)),
        "approx_kl": float(np.mean(kl_means)),
        "clip_frac": float(np.mean(clip_fracs)),
        "early_stop": early_stop,
    }


# ── main ───────────────────────────────────────────────────────────────


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--total-steps", type=int, default=1_000_000,
                   help="Total environment steps across ALL parallel envs.")
    p.add_argument("--n-envs", type=int, default=8)
    p.add_argument("--horizon", type=int, default=512,
                   help="Steps per env per rollout (batch = horizon * n_envs).")
    p.add_argument("--epochs", type=int, default=10)
    p.add_argument("--batch-size", type=int, default=512)
    p.add_argument("--lr", type=float, default=3e-4)
    p.add_argument("--clip", type=float, default=0.2)
    p.add_argument("--ent-coef", type=float, default=0.01)
    p.add_argument("--ent-coef-final", type=float, default=0.001)
    p.add_argument("--vf-coef", type=float, default=0.5)
    p.add_argument("--target-kl", type=float, default=0.03)
    p.add_argument("--action-repeat", type=int, default=4)
    p.add_argument("--action-smoothness-coef", type=float, default=0.002)
    p.add_argument("--max-steps", type=int, default=600,
                   help="Physics ticks per episode.")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--save", type=str, default=None)
    p.add_argument(
        "--num-active-opponents", type=int, default=2,
        help="Scripted red robots that actually move. 0 = no opponents "
             "(training stage 1), 2 = no goalie, 3 = full strength.",
    )
    p.add_argument(
        "--randomize-ball", action="store_true",
        help="Spawn the ball uniformly over the field (minus wall margin) "
             "each reset, instead of fixed center kickoff. Use this when "
             "training a scoring policy — centered spawns let the policy "
             "memorize a single trajectory.",
    )
    p.add_argument(
        "--robot-to-ball-scale", type=float, default=None,
        help="Override reward shaping coefficient for 'closest robot "
             "approaches ball'. Default (None) keeps rewards.py default "
             "of 0.1. Boost to ~0.3 when ball spawns randomly so the "
             "navigation signal dominates before the policy has learned "
             "to kick.",
    )
    p.add_argument(
        "--resume", type=str, default=None,
        help="Warm-start from an existing checkpoint (model weights + "
             "obs_rms stats). LR/entropy schedules restart for this run's "
             "duration, so it behaves like curriculum-style fine-tuning."
    )
    p.add_argument("--smoke", action="store_true")
    args = p.parse_args()

    if args.smoke:
        args.total_steps = 4096
        args.n_envs = 4
        args.horizon = 128
        args.epochs = 2
        args.batch_size = 128

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    env_fns = [
        make_env_factory(
            seed=args.seed + i,
            action_repeat=args.action_repeat,
            action_smoothness_coef=args.action_smoothness_coef,
            max_steps=args.max_steps,
            num_active_opponents=args.num_active_opponents,
            randomize_ball_spawn=args.randomize_ball,
            robot_to_ball_scale=args.robot_to_ball_scale,
        )
        for i in range(args.n_envs)
    ]
    envs = SubprocVecEnv(env_fns)
    obs, _ = envs.reset(seeds=[args.seed + i for i in range(args.n_envs)])

    device = torch.device("cpu")
    model = ActorCritic(OBS_DIM, ACT_DIM).to(device)
    optimizer = optim.Adam(model.parameters(), lr=args.lr, eps=1e-5)

    obs_rms = RunningMeanStd((OBS_DIM,))
    ret_norm = ReturnNormalizer(args.n_envs, gamma=0.99)

    if args.resume is not None:
        ckpt = torch.load(args.resume, map_location=device, weights_only=False)
        model.load_state_dict(ckpt["model_state_dict"])
        obs_rms.mean = np.asarray(ckpt["obs_rms_mean"])
        obs_rms.var = np.asarray(ckpt["obs_rms_var"])
        obs_rms.count = float(ckpt["obs_rms_count"])
        print(f"[train] resumed from {args.resume}")

    steps_per_update = args.horizon * args.n_envs
    n_updates = max(1, args.total_steps // steps_per_update)
    print(
        f"[train] n_envs={args.n_envs}  horizon={args.horizon}  "
        f"steps/update={steps_per_update}  updates={n_updates}  "
        f"action_repeat={args.action_repeat}  "
        f"physics_ticks_total={args.total_steps * args.action_repeat:,}"
    )

    t_start = time.perf_counter()
    global_step = 0
    try:
        for upd in range(1, n_updates + 1):
            frac = 1.0 - (upd - 1) / n_updates
            cur_lr = args.lr * frac
            cur_ent = args.ent_coef_final + (args.ent_coef - args.ent_coef_final) * frac
            for g in optimizer.param_groups:
                g["lr"] = cur_lr

            rollout, obs, roll_stats = collect_rollout(
                envs, model, obs, args.horizon, obs_rms, ret_norm, device
            )
            up_stats = ppo_update(
                model, optimizer, rollout,
                epochs=args.epochs,
                batch_size=args.batch_size,
                clip=args.clip,
                ent_coef=cur_ent,
                vf_coef=args.vf_coef,
                target_kl=args.target_kl,
                device=device,
            )

            global_step += steps_per_update
            elapsed = time.perf_counter() - t_start
            sps = global_step / elapsed
            print(
                f"[upd {upd:4d}/{n_updates}] step={global_step:>8d}  "
                f"sps={sps:6.0f}  "
                f"ep_ret={roll_stats['ep_return_mean']:+.3f}  "
                f"ep_len={roll_stats['ep_length_mean']:.0f}  "
                f"goals=+{roll_stats['goals_for']}/-{roll_stats['goals_against']}  "
                f"pi_l={up_stats['policy_loss']:+.4f}  "
                f"v_l={up_stats['value_loss']:.4f}  "
                f"ent={up_stats['entropy']:+.3f}  "
                f"kl={up_stats['approx_kl']:+.4f}  "
                f"clip={up_stats['clip_frac']:.2f}  "
                f"lr={cur_lr:.1e}"
                + ("  [KL-stop]" if up_stats["early_stop"] else "")
            )
    finally:
        envs.close()

    if args.save:
        os.makedirs(
            os.path.dirname(os.path.abspath(args.save)) or ".", exist_ok=True
        )
        torch.save(
            {
                "model_state_dict": model.state_dict(),
                "obs_dim": OBS_DIM,
                "act_dim": ACT_DIM,
                "obs_rms_mean": obs_rms.mean,
                "obs_rms_var": obs_rms.var,
                "obs_rms_count": obs_rms.count,
                "args": vars(args),
            },
            args.save,
        )
        print(f"[train] saved → {args.save}")


if __name__ == "__main__":
    main()
