"""
Subprocess-based vectorized environment.

Each env runs in its own child process; commands and results flow over
``multiprocessing.Pipe``. We use the ``fork`` start method so worker
functions can be lambdas / closures (no need to make every env factory
top-level picklable). On macOS this is generally safe because pymunk +
numpy don't fork-unsafe-ly hold native locks; if you ever see hangs,
switch the context to ``spawn`` and convert the env factories into
top-level callables.

Auto-reset semantics match gymnasium.vector: when a sub-env reports
terminated or truncated, the worker immediately calls ``env.reset()``
and returns the FRESH observation. The original final observation is
stashed in ``info["final_observation"]`` for any algorithm that needs
to bootstrap a value off it. Episode return / length are also rolled
up in ``info["episode"]`` for logging.
"""

from __future__ import annotations

import multiprocessing as mp
from typing import Callable, Sequence

import numpy as np


def _worker(remote, env_fn: Callable) -> None:
    env = env_fn()
    ep_return = 0.0
    ep_length = 0
    try:
        while True:
            cmd, data = remote.recv()
            if cmd == "step":
                obs, r, term, trunc, info = env.step(data)
                ep_return += float(r)
                ep_length += 1
                if term or trunc:
                    info = dict(info)
                    info["final_observation"] = obs
                    info["episode"] = {
                        "return": ep_return,
                        "length": ep_length,
                    }
                    obs, _ = env.reset()
                    ep_return = 0.0
                    ep_length = 0
                remote.send((obs, r, term, trunc, info))
            elif cmd == "reset":
                obs, info = env.reset(seed=data)
                ep_return = 0.0
                ep_length = 0
                remote.send((obs, info))
            elif cmd == "spec":
                remote.send((env.observation_space, env.action_space))
            elif cmd == "close":
                remote.close()
                return
            else:
                raise ValueError(f"Unknown command: {cmd}")
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        try:
            env.close()
        except Exception:
            pass


class SubprocVecEnv:
    """Vector env running each sub-env in a child process."""

    def __init__(self, env_fns: Sequence[Callable]) -> None:
        self.n_envs = len(env_fns)
        # ``fork`` lets us pass closures; switch to ``spawn`` if you hit
        # native-thread-related instability inside a worker.
        ctx = mp.get_context("fork")

        self.remotes: list = []
        self.workers: list = []
        for fn in env_fns:
            parent_end, child_end = ctx.Pipe()
            p = ctx.Process(target=_worker, args=(child_end, fn), daemon=True)
            p.start()
            child_end.close()
            self.remotes.append(parent_end)
            self.workers.append(p)

        self.remotes[0].send(("spec", None))
        self.observation_space, self.action_space = self.remotes[0].recv()

    # ── public API ─────────────────────────────────────────────────────

    def reset(self, seeds: Sequence[int | None] | None = None):
        if seeds is None:
            seeds = [None] * self.n_envs
        for r, s in zip(self.remotes, seeds, strict=True):
            r.send(("reset", s))
        results = [r.recv() for r in self.remotes]
        obs = np.stack([o for o, _ in results]).astype(np.float32)
        infos = [i for _, i in results]
        return obs, infos

    def step(self, actions: np.ndarray):
        for r, a in zip(self.remotes, actions, strict=True):
            r.send(("step", a))
        results = [r.recv() for r in self.remotes]
        obs = np.stack([o for o, _, _, _, _ in results]).astype(np.float32)
        rewards = np.array(
            [r for _, r, _, _, _ in results], dtype=np.float32
        )
        terminated = np.array(
            [t for _, _, t, _, _ in results], dtype=bool
        )
        truncated = np.array(
            [t for _, _, _, t, _ in results], dtype=bool
        )
        infos = [i for _, _, _, _, i in results]
        return obs, rewards, terminated, truncated, infos

    def close(self) -> None:
        for r in self.remotes:
            try:
                r.send(("close", None))
            except (BrokenPipeError, EOFError):
                pass
        for w in self.workers:
            w.join(timeout=2.0)
            if w.is_alive():
                w.terminate()
