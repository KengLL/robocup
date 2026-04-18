"""
Sensor-realistic observation wrapper for SoccerEnv.

Corrupts the perfect ground-truth dict that SoccerEnv.get_obs() returns
into something that resembles what the real robot would actually see
through its AprilTag pipeline:

  - position noise   : zero-mean Gaussian on (x, y) and angle
  - latency          : returned obs is N ticks behind reality (vision lag)
  - dropout          : per-entity, the wrapper occasionally fails to
                       update and returns the last successfully observed
                       pose (e.g. tag occluded / blurred)
  - omega masking    : opponents' angular velocity is unobservable
                       (we only see opponents through vision; we do not
                       have their gyros). It is removed from the obs.
  - velocity estimate: vision gives positions, not velocities — the
                       wrapper estimates vx/vy by finite-differencing
                       its own delayed/noisy outputs (NOT the ground
                       truth) so the policy sees the same noise the
                       real estimator would see.

This is the largest sim-to-real gap once the physics is reasonable, so
training against the corrupted obs (not the ground-truth obs) is what
makes the policy robust on real hardware.
"""

from __future__ import annotations

import random
from collections import deque
from dataclasses import dataclass, field
from typing import Any


@dataclass
class ObservationWrapperConfig:
    # AprilTag pose noise (1-sigma).
    pos_noise_std: float = 0.02      # meters; ~2 cm typical for a calibrated tag
    angle_noise_std: float = 0.03    # radians; ~1.7 deg
    ball_pos_noise_std: float = 0.015  # meters; ball is small but tracked well

    # Vision latency, expressed in physics ticks (DT = 1/60 s).
    # 30 ms ≈ 2 ticks, 80 ms ≈ 5 ticks. Each reset() picks a value in this
    # range so the policy must tolerate the whole envelope.
    latency_ticks_min: int = 2
    latency_ticks_max: int = 5

    # Probability a given entity (ball / one robot) fails to update on a
    # given tick. Independent across entities. When an entity drops, the
    # wrapper returns its last successful pose with zero estimated
    # velocity (matches what an EKF would do under prolonged dropout).
    dropout_prob: float = 0.05

    # Teams. Blue = robots 0,1,2; red = robots 3,4,5. The "our" team is
    # the one we control; their omega comes from on-board gyros and is
    # NOT masked. Opponent omega is masked (set to 0.0).
    our_color: str = "blue"
    blue_ids: tuple[int, ...] = (0, 1, 2)
    red_ids: tuple[int, ...] = (3, 4, 5)


@dataclass
class _Track:
    """Last successfully-observed pose for one entity, plus the previous
    one for finite-difference velocity estimation."""

    last: dict[str, float] | None = None
    prev: dict[str, float] | None = None


class ObservationWrapper:
    """
    Stateful corrupter. One instance per environment. Call ``reset()``
    whenever the underlying SoccerEnv resets, then call ``observe(obs)``
    once per physics tick with the ground-truth dict.
    """

    def __init__(
        self,
        config: ObservationWrapperConfig | None = None,
        rng: random.Random | None = None,
    ) -> None:
        self.cfg = config or ObservationWrapperConfig()
        self.rng = rng or random.Random()
        self._buffer: deque[dict] = deque()
        self._latency_ticks: int = self.cfg.latency_ticks_min
        self._tracks: dict[str, _Track] = {}

    # ── lifecycle ──────────────────────────────────────────────────────

    def reset(self, seed: int | None = None) -> None:
        if seed is not None:
            self.rng = random.Random(seed)
        self._latency_ticks = self.rng.randint(
            self.cfg.latency_ticks_min, self.cfg.latency_ticks_max
        )
        self._buffer.clear()
        self._tracks = {"ball": _Track()}
        for rid in (*self.cfg.blue_ids, *self.cfg.red_ids):
            self._tracks[f"robot_{rid}"] = _Track()

    # ── per-tick ───────────────────────────────────────────────────────

    def observe(self, true_obs: dict) -> dict:
        """
        Push the latest ground-truth obs into the latency buffer; return
        a corrupted obs lagged by ``_latency_ticks``. Until the buffer
        is full, returns the oldest entry available (so the first few
        ticks aren't garbage but ARE stale from t=0).
        """
        self._buffer.append(true_obs)
        # Hold one extra so popleft happens after we picked the lagged one.
        while len(self._buffer) > self._latency_ticks + 1:
            self._buffer.popleft()
        lagged = self._buffer[0]
        return self._corrupt(lagged, true_obs["t"])

    # ── internals ──────────────────────────────────────────────────────

    def _corrupt(self, obs: dict, current_t: float) -> dict:
        out: dict[str, Any] = {
            "t": current_t,                     # the policy knows the real clock
            "obs_t": obs["t"],                  # timestamp the obs was *captured* at
            "score": dict(obs["score"]),
            "latency_ticks": self._latency_ticks,
        }

        # Ball.
        ball_track = self._tracks["ball"]
        if self.rng.random() < self.cfg.dropout_prob:
            updated = False
        else:
            updated = True
            noisy = {
                "x": obs["ball"]["x"] + self.rng.gauss(0.0, self.cfg.ball_pos_noise_std),
                "y": obs["ball"]["y"] + self.rng.gauss(0.0, self.cfg.ball_pos_noise_std),
                "t": obs["t"],
            }
            ball_track.prev = ball_track.last
            ball_track.last = noisy
        out["ball"] = self._track_to_obs(ball_track, vel=True, has_angle=False)

        # Robots.
        out["robots"] = {}
        for rid_str, robot in obs["robots"].items():
            rid = int(rid_str)
            track = self._tracks[f"robot_{rid}"]
            if self.rng.random() < self.cfg.dropout_prob:
                pass  # leave last/prev untouched
            else:
                noisy = {
                    "x": robot["x"] + self.rng.gauss(0.0, self.cfg.pos_noise_std),
                    "y": robot["y"] + self.rng.gauss(0.0, self.cfg.pos_noise_std),
                    "angle": robot["angle"]
                    + self.rng.gauss(0.0, self.cfg.angle_noise_std),
                    "t": obs["t"],
                }
                track.prev = track.last
                track.last = noisy

            entry = self._track_to_obs(track, vel=True, has_angle=True)

            # On-board gyro is only available for our own team.
            is_ours = self._is_our_robot(rid)
            entry["omega"] = robot["omega"] if is_ours else 0.0
            entry["omega_observed"] = is_ours
            out["robots"][rid_str] = entry

        return out

    def _track_to_obs(
        self, track: _Track, *, vel: bool, has_angle: bool
    ) -> dict[str, float]:
        if track.last is None:
            # First few ticks before any successful update: emit zeros
            # rather than crashing. The policy has the t/obs_t channels
            # to recognize this state.
            base = {"x": 0.0, "y": 0.0}
            if has_angle:
                base["angle"] = 0.0
            if vel:
                base["vx"] = 0.0
                base["vy"] = 0.0
            return base

        base = {"x": track.last["x"], "y": track.last["y"]}
        if has_angle:
            base["angle"] = track.last["angle"]
        if vel:
            if track.prev is None or track.last["t"] == track.prev["t"]:
                base["vx"] = 0.0
                base["vy"] = 0.0
            else:
                dt = track.last["t"] - track.prev["t"]
                base["vx"] = (track.last["x"] - track.prev["x"]) / dt
                base["vy"] = (track.last["y"] - track.prev["y"]) / dt
        return base

    def _is_our_robot(self, rid: int) -> bool:
        if self.cfg.our_color == "blue":
            return rid in self.cfg.blue_ids
        return rid in self.cfg.red_ids
