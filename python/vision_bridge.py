"""Live camera state from robocup_testing's track-combined, in sim coordinates.

A background thread keeps only the newest tracker doc. Each tracked object is
fresh / coast / lost by the age of its last accepted sighting; apply_to_world()
turns that into kinematic overrides on a PymunkWorld.
"""

from __future__ import annotations

import json
import math
import os
import sys
import threading
import time
import urllib.error
import urllib.request
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
from config import (  # noqa: E402
    BALL_DAMP,
    FIELD_H,
    FIELD_IS_DEFAULT,
    FIELD_W,
    LINEAR_DAMP,
    MAX_BALL_SPEED,
    NUM_ROBOTS,
    ROBOT_MAX_ACCEL,
    ROBOT_MAX_SPEED,
    VISION_BALL_COAST_S,
    VISION_BLEND_TAU,
    VISION_FIELD_MARGIN,
    VISION_FLIP_X,
    VISION_FLIP_Y,
    VISION_FRESH_S,
    VISION_GATE_MARGIN,
    VISION_GATE_RELOCK,
    VISION_REPLAY_GAP_CAP,
    VISION_ROBOT_COAST_S,
    VISION_SNAP_ANGLE,
    VISION_SNAP_DIST,
    VISION_STATS_WINDOW,
    VISION_TRACKER_COAST,
    VISION_VEL_NOISE,
)

# predict: unseen, riding the tracker's own coast inside the reach circle.
FRESH, PREDICT, COAST, LOST = "fresh", "predict", "coast", "lost"

# Latency above this is treated as clock skew between machines, not real delay.
_MAX_PLAUSIBLE_LATENCY = 2.0  # seconds
_LINK_TIMEOUT = 1.0  # seconds without a new doc before the link reads as down


@dataclass
class Sighting:
    """One object as the sim should see it right now, sim coordinates."""

    mode: str
    age: float  # seconds since last accepted sighting, inf if never seen
    x: float = 0.0
    y: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    angle: float = 0.0  # rad
    omega: float = 0.0  # rad/s


@dataclass
class _Track:
    vmax: float  # m/s, fastest the object can move
    accel: float | None  # m/s^2, most its drive can add; None: kicks set any velocity
    damp: float  # 1/s, velocity decay (dv/dt = a - damp * v)
    coast_s: float
    x: float = 0.0  # real field, meters
    y: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    theta: float = 0.0  # rad
    omega: float = 0.0  # rad/s
    t_meas: float | None = None  # wall-clock capture time of last accepted sighting
    # Tracker's Kalman coast while unseen: (x, y, vx, vy, theta, omega), clamped on arrival.
    pred: tuple[float, ...] | None = None
    t_pred: float | None = None
    predicted: int = 0  # total coasted entries used
    rejects: int = 0  # current streak
    rejected: int = 0  # total
    seen: deque = field(default_factory=lambda: deque(maxlen=VISION_STATS_WINDOW))

    def reach(self, t: float) -> tuple[float, float, float]:
        """(cx, cy, r): where the object can be at t, from the last accepted sighting.

        Robot: dv/dt = a - damp*v with |a| <= accel. Its measured velocity carries
        it k = (1 - e^(-damp*g)) / damp seconds' worth; thrust in a fixed direction
        adds at most accel * (g - k) / damp on top, in any direction. Ball: a kick
        can set any velocity up to vmax at any instant, so vmax * g around it.
        """
        g = max(0.0, t - self.t_meas)
        if self.accel is None:
            return self.x, self.y, self.vmax * g + VISION_GATE_MARGIN
        k = (1.0 - math.exp(-self.damp * g)) / self.damp
        r = self.accel * (g - k) / self.damp + VISION_VEL_NOISE * g + VISION_GATE_MARGIN
        return self.x + self.vx * k, self.y + self.vy * k, r

    def clamp(self, x: float, y: float, t: float) -> tuple[float, float]:
        cx, cy, r = self.reach(t)
        dx, dy = x - cx, y - cy
        d = math.hypot(dx, dy)
        if d <= r:
            return x, y
        return cx + dx * r / d, cy + dy * r / d

    def gate(self, x: float, y: float, t: float) -> bool:
        if self.t_meas is None or t - self.t_meas > self.coast_s:
            return True
        cx, cy, r = self.reach(t)
        if math.hypot(x - cx, y - cy) <= r:
            return True
        self.rejects += 1
        self.rejected += 1
        # A run of rejects means the old track was wrong, not the camera.
        return self.rejects >= VISION_GATE_RELOCK


def _cap(vx: float, vy: float, vmax: float) -> tuple[float, float]:
    s = math.hypot(vx, vy)
    return (vx, vy) if s <= vmax else (vx * vmax / s, vy * vmax / s)


def parse_tag_map(s: str) -> dict[int, int] | None:
    """'0:0,4:1' -> {tag 0: robot 0, tag 4: robot 1}. 'auto' -> None."""
    if s.strip().lower() == "auto":
        return None
    out: dict[int, int] = {}
    for pair in s.split(","):
        if pair.strip():
            tag, rid = pair.split(":")
            out[int(tag)] = int(rid)
    return out


def parse_ids(s: str) -> set[int]:
    return {int(x) for x in s.split(",") if x.strip()}


# ── sources ──────────────────────────────────────────────────────────────────
# Each runs on the reader thread and calls emit(doc) per tracker frame.


def _http_source(url: str, emit: Callable, stop: threading.Event, status: dict) -> None:
    period = 1.0 / 60.0  # poll faster than the camera; duplicates are dropped by seq/timestamp
    while not stop.is_set():
        t0 = time.perf_counter()
        try:
            with urllib.request.urlopen(url, timeout=0.5) as resp:
                doc = json.loads(resp.read())
            if doc:
                emit(doc)
            status["error"] = None
        except (urllib.error.URLError, OSError, ValueError) as e:
            if status.get("error") is None:
                print(f"[VisionBridge] {url}: {e}")
            status["error"] = str(e)
            stop.wait(0.5)
            continue
        stop.wait(max(0.0, period - (time.perf_counter() - t0)))


def _zmq_source(endpoint: str, emit: Callable, stop: threading.Event, status: dict) -> None:
    import zmq

    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)
    # CONFLATE must be set before connect; keeps only the newest frame queued.
    sub.setsockopt(zmq.CONFLATE, 1)
    sub.setsockopt(zmq.RCVTIMEO, 200)
    sub.setsockopt_string(zmq.SUBSCRIBE, "")
    sub.connect(endpoint)
    try:
        while not stop.is_set():
            try:
                emit(json.loads(sub.recv_string()))
            except zmq.Again:
                continue
    finally:
        sub.close(0)


def _replay_source(
    path: str, speed: float, emit: Callable, stop: threading.Event, status: dict
) -> None:
    # Recorded timestamps are rewritten to "now" so latency/age math sees a live feed.
    while not stop.is_set():
        prev_t = None
        # Pace against an absolute schedule: per-frame sleeps overshoot on
        # Windows (~15 ms timer) and replay drifted to ~75% speed.
        wall0, rec_elapsed = time.perf_counter(), 0.0
        with open(path) as f:
            for line in f:
                if stop.is_set():
                    return
                if not line.strip():
                    continue
                doc = json.loads(line)
                t = doc.get("t_capture", doc.get("timestamp"))
                if prev_t is not None and t is not None:
                    rec_elapsed += min(max(0.0, t - prev_t), VISION_REPLAY_GAP_CAP)
                    wait = wall0 + rec_elapsed / speed - time.perf_counter()
                    if wait > 0:
                        stop.wait(wait)
                prev_t = t
                # Keep the recorded capture->publish delay so replay latency is real.
                now = time.time()
                delay = 0.0
                if "t_capture" in doc and "t_publish" in doc:
                    delay = doc["t_publish"] - doc["t_capture"]
                doc["timestamp"] = doc["t_publish"] = now
                doc["t_capture"] = now - delay
                emit(doc)
        print(f"[VisionBridge] replay reached end of {path}, looping")
        status["loops"] = status.get("loops", 0) + 1


# ── bridge ───────────────────────────────────────────────────────────────────


class VisionBridge:
    def __init__(
        self,
        source: str,
        tag_map: dict[int, int] | None,
        replay_speed: float = 1.0,
        ignore_tags: set[int] | None = None,
        flip_x: bool = VISION_FLIP_X,
        flip_y: bool = VISION_FLIP_Y,
    ) -> None:
        self.source = source
        # None: each new tag id gets the lowest free robot, kept for the session.
        self.auto = tag_map is None
        self.tag_map = {t: r for t, r in (tag_map or {}).items() if 0 <= r < NUM_ROBOTS}
        self.ignore_tags = set(ignore_tags or ())
        self.flip_x, self.flip_y = flip_x, flip_y
        self._warned_tags: set[int] = set()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._status: dict[str, Any] = {"error": None}

        self.ball = _Track(MAX_BALL_SPEED, None, BALL_DAMP, VISION_BALL_COAST_S)
        self.robots = {rid: self._new_robot_track() for rid in self.tag_map.values()}

        self.field_w, self.field_h = 1.2, 0.8  # tracker default, replaced by each doc
        # Where the field's first corner sits in tracker coordinates (custom origin).
        self.x0, self.y0 = 0.0, 0.0
        self._warned_aspect = False
        self._last_key: Any = None
        self._last_seq: int | None = None
        self._run_id: str | None = None
        self.tracker_stats: dict[str, Any] = {}
        self._last_cap: float | None = None
        self._last_recv = 0.0
        self.frames = 0
        self.dupes = 0
        self.transport_drops = 0
        self.fps = 0.0
        self.latency_ms = 0.0

        if source.startswith(("http://", "https://")):
            self.kind = "http"
            target = lambda: _http_source(source, self._on_doc, self._stop, self._status)
        elif source.startswith(("tcp://", "ipc://", "zmq://")):
            self.kind = "zmq"
            ep = source.replace("zmq://", "tcp://", 1)
            target = lambda: _zmq_source(ep, self._on_doc, self._stop, self._status)
        else:
            if not os.path.isfile(source):
                raise FileNotFoundError(f"vision source not a URL or file: {source}")
            self.kind = "replay"
            target = lambda: _replay_source(
                source, replay_speed, self._on_doc, self._stop, self._status
            )
        self._thread = threading.Thread(target=target, daemon=True, name="VisionBridge")

    def start(self) -> VisionBridge:
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()

    @property
    def mirrored_rids(self) -> set[int]:
        with self._lock:
            return set(self.robots)

    @staticmethod
    def _new_robot_track() -> _Track:
        return _Track(ROBOT_MAX_SPEED, ROBOT_MAX_ACCEL, LINEAR_DAMP, VISION_ROBOT_COAST_S)

    def _robot_for_tag(self, tag: dict) -> int | None:
        tid = int(tag.get("id", -1))
        if tid in self.ignore_tags:
            return None
        rid = self.tag_map.get(tid)
        if rid is not None or tid in self._warned_tags:
            return rid
        if not self.auto:
            print(f"[VisionBridge] tag {tid} seen but not in --tag-map, ignoring it")
            self._warned_tags.add(tid)
            return None
        # Only claim a robot for a real on-field sighting, not a coasting entry.
        x, y, m = tag.get("x", -1.0), tag.get("y", -1.0), VISION_FIELD_MARGIN
        if not (tag.get("visible") and -m <= x <= self.field_w + m and -m <= y <= self.field_h + m):
            return None
        free = [r for r in range(NUM_ROBOTS) if r not in self.robots]
        if not free:
            print(f"[VisionBridge] tag {tid} seen but all {NUM_ROBOTS} robots are taken")
            self._warned_tags.add(tid)
            return None
        rid = free[0]
        self.tag_map[tid] = rid
        self.robots[rid] = self._new_robot_track()
        print(f"[VisionBridge] tag {tid} -> robot {rid}")
        return rid

    # ── ingest (reader thread) ───────────────────────────────────────────────

    def _on_doc(self, doc: dict[str, Any]) -> None:
        t_recv = time.time()
        # sim-feed branch says "seq", main says "frame"; either is a per-frame counter.
        seq = doc.get("seq", doc.get("frame"))
        run_id = doc.get("run_id")
        key = (run_id, seq) if seq is not None else doc.get("timestamp")
        with self._lock:
            # HTTP polling outruns the camera, so the same frame shows up twice.
            if key is not None and key == self._last_key:
                self.dupes += 1
                return
            self._last_key = key
            # Tracker restart (new run_id, or seq going backwards on sim-feed which has
            # no run_id; also a looping replay): not a gap, and old tracks can't gate.
            restarted = run_id != self._run_id and self._run_id is not None
            if seq is not None and self._last_seq is not None and seq < self._last_seq:
                restarted = True
            if restarted:
                for tr in (self.ball, *self.robots.values()):
                    tr.t_meas, tr.rejects = None, 0
                    tr.pred = tr.t_pred = None
            if restarted or run_id != self._run_id:
                self._run_id = run_id
                self._last_seq = None
            if seq is not None:
                if self._last_seq is not None and seq > self._last_seq + 1:
                    self.transport_drops += seq - self._last_seq - 1
                self._last_seq = seq
            if isinstance(doc.get("stats"), dict):
                self.tracker_stats = doc["stats"]

            t_cap = doc.get("t_capture", doc.get("timestamp", t_recv))
            lat = t_recv - t_cap
            if lat < -0.05 or lat > _MAX_PLAUSIBLE_LATENCY:
                t_cap, lat = t_recv, 0.0  # clocks disagree, trust arrival time
            self.latency_ms = 0.9 * self.latency_ms + 0.1 * lat * 1000.0
            if self._last_cap is not None and t_cap > self._last_cap:
                self.fps = 0.9 * self.fps + 0.1 / (t_cap - self._last_cap)
            self._last_cap = t_cap
            self._last_recv = t_recv
            self.frames += 1

            fld = doc.get("field") or {}
            self.field_w = float(fld.get("width", self.field_w))
            self.field_h = float(fld.get("height", self.field_h))
            # Logs from before custom origins have no x0/y0: corner origin.
            self.x0 = float(fld.get("x0", 0.0))
            self.y0 = float(fld.get("y0", 0.0))
            if not self._warned_aspect and not FIELD_IS_DEFAULT:
                # True-scale sim: _to_sim() should be 1:1, so the sizes must agree.
                if max(abs(self.field_w - FIELD_W), abs(self.field_h - FIELD_H)) > 0.005:
                    print(
                        f"[VisionBridge] WARNING tracker field {self.field_w}x{self.field_h} "
                        + f"!= sim --field {FIELD_W}x{FIELD_H}; positions will be rescaled"
                    )
                self._warned_aspect = True
            elif not self._warned_aspect:
                a_real, a_sim = self.field_w / self.field_h, FIELD_W / FIELD_H
                if abs(a_real / a_sim - 1.0) > 0.02:
                    print(
                        f"[VisionBridge] WARNING real field {self.field_w}x{self.field_h} "
                        + f"and sim {FIELD_W}x{FIELD_H} differ in aspect; headings will skew"
                    )
                self._warned_aspect = True

            seen_rids: set[int] = set()
            for tag in doc.get("tags") or []:
                tag = self._to_corner(tag)
                rid = self._robot_for_tag(tag)
                if rid is None or rid in seen_rids:
                    continue
                seen_rids.add(rid)
                self._ingest(self.robots[rid], tag, t_cap, heading=True)
            for rid, tr in self.robots.items():
                if rid not in seen_rids:
                    tr.seen.append(False)

            ball = doc.get("ball")
            if ball:
                self._ingest(self.ball, self._to_corner(ball), t_cap, heading=False)
            else:
                self.ball.seen.append(False)

    def _to_corner(self, e: dict) -> dict:
        # Tracker x/y are relative to its chosen origin; everything here (bounds,
        # gate, flips, scaling) works corner-relative, so shift once on the way in.
        e = dict(e)
        if "x" in e:
            e["x"] = float(e["x"]) - self.x0
        if "y" in e:
            e["y"] = float(e["y"]) - self.y0
        return e

    def _ingest(self, tr: _Track, e: dict, t_cap: float, heading: bool) -> None:
        x, y = float(e.get("x", 0.0)), float(e.get("y", 0.0))
        vx, vy = float(e.get("vx", 0.0)), float(e.get("vy", 0.0))
        theta = math.radians(float(e.get("theta_deg", 0.0))) if heading else 0.0
        omega = math.radians(float(e.get("omega_deg", 0.0))) if heading else 0.0
        m = VISION_FIELD_MARGIN
        visible, lost = e.get("visible", False), e.get("lost", False)
        real = visible and not lost and -m <= x <= self.field_w + m and -m <= y <= self.field_h + m
        ok = real and tr.gate(x, y, t_cap)
        tr.seen.append(bool(ok))
        if ok:
            tr.x, tr.y = x, y
            tr.vx, tr.vy = _cap(vx, vy, tr.vmax)
            if heading:
                tr.theta, tr.omega = theta, omega
            tr.t_meas = t_cap
            tr.rejects = 0
            tr.pred = tr.t_pred = None
            return
        # Tracker coasting on its Kalman filter: use it, but only from a real fix
        # we still trust, and never outside the reach circle or the field. Unbounded,
        # that coast once carried the ball 6 m off a 0.8 m field.
        if (
            not VISION_TRACKER_COAST
            or visible
            or lost
            or tr.t_meas is None
            or t_cap - tr.t_meas >= tr.coast_s
        ):
            return
        x, y = tr.clamp(x, y, t_cap)
        x = min(max(x, -m), self.field_w + m)
        y = min(max(y, -m), self.field_h + m)
        vx, vy = _cap(vx, vy, tr.vmax)
        if not heading:
            theta, omega = tr.theta, tr.omega
        tr.pred, tr.t_pred = (x, y, vx, vy, theta, omega), t_cap
        tr.predicted += 1

    # ── output (sim thread) ──────────────────────────────────────────────────

    def _to_sim(
        self, tr: _Track, st: tuple[float, ...], t0: float, now: float, age: float, mode: str
    ) -> Sighting:
        # Dead-reckon state st (taken at t0) over capture latency plus the time
        # since that frame, then keep it inside the reach circle.
        x, y, vx, vy, theta, omega = st
        dt = max(0.0, now - t0) if mode in (FRESH, PREDICT) else 0.0
        vx, vy = _cap(vx, vy, tr.vmax)
        x, y = tr.clamp(x + vx * dt, y + vy * dt, now)
        theta += omega * dt
        # TODO(hardware): on the default 9 x 6 field this inflates speeds (x4.7 for a
        # 1.93 m table); run_pipeline.py --field makes the sim true-size and this 1:1.
        sx, sy = FIELD_W / self.field_w, FIELD_H / self.field_h
        x, y, vx, vy = x * sx, y * sy, vx * sx, vy * sy
        if self.flip_x:
            x, vx, theta, omega = FIELD_W - x, -vx, math.pi - theta, -omega
        if self.flip_y:
            y, vy, theta, omega = FIELD_H - y, -vy, -theta, -omega
        return Sighting(mode, age, x, y, vx, vy, theta, omega)

    def _sighting(self, tr: _Track, now: float) -> Sighting:
        if tr.t_meas is None:
            return Sighting(LOST, math.inf)
        age = max(0.0, now - tr.t_meas)
        meas = (tr.x, tr.y, tr.vx, tr.vy, tr.theta, tr.omega)
        if age < VISION_FRESH_S:
            return self._to_sim(tr, meas, tr.t_meas, now, age, FRESH)
        if age < tr.coast_s and tr.pred is not None and now - tr.t_pred < VISION_FRESH_S:
            return self._to_sim(tr, tr.pred, tr.t_pred, now, age, PREDICT)
        return self._to_sim(tr, meas, tr.t_meas, now, age, COAST if age < tr.coast_s else LOST)

    def poll(self, now: float | None = None) -> dict[str, Any]:
        now = time.time() if now is None else now
        with self._lock:
            robots = {rid: self._sighting(tr, now) for rid, tr in self.robots.items()}
            ball = self._sighting(self.ball, now)

            def drop_pct(tr: _Track) -> float | None:
                return 100.0 * (1.0 - sum(tr.seen) / len(tr.seen)) if tr.seen else None

            link_age = now - self._last_recv if self._last_recv else math.inf
            stats = {
                "source": self.kind,
                "link_up": link_age < _LINK_TIMEOUT,
                "link_age_ms": None if math.isinf(link_age) else round(link_age * 1000.0),
                "fps": round(self.fps, 1),
                "latency_ms": round(self.latency_ms, 1),
                "frames": self.frames,
                "dupes": self.dupes,
                "transport_drops": self.transport_drops,
                "rejects": self.ball.rejected + sum(t.rejected for t in self.robots.values()),
                "predicted": self.ball.predicted + sum(t.predicted for t in self.robots.values()),
                "drop_pct": {
                    "ball": drop_pct(self.ball),
                    **{str(rid): drop_pct(tr) for rid, tr in self.robots.items()},
                },
                "error": self._status.get("error"),
                "tags": {str(r): t for t, r in self.tag_map.items() if r in self.robots},
                # Tracker's own view: capture_dropped = camera frames it was too slow for.
                "tracker": {
                    k: self.tracker_stats[k]
                    for k in ("fps", "tag_ms", "ball_ms", "latency_ms", "capture_dropped")
                    if k in self.tracker_stats
                },
            }
        return {"robots": robots, "ball": ball, "stats": stats}


def apply_to_world(world: Any, snap: dict[str, Any]) -> None:
    """Fresh / predict: camera (or tracker's coast) drives the body. Coast: physics
    carries it. Lost: robot freezes in place, ball goes back to the sim."""
    for rid, s in snap["robots"].items():
        if s.mode in (FRESH, PREDICT):
            world.drive_robot(
                rid, (s.x, s.y), (s.vx, s.vy), s.angle, s.omega,
                VISION_BLEND_TAU, VISION_SNAP_DIST, VISION_SNAP_ANGLE,
            )
        elif s.mode == COAST:
            world.coast_robot(rid)
        else:
            world.freeze_robot(rid)
    b = snap["ball"]
    if b.mode in (FRESH, PREDICT):
        world.drive_ball((b.x, b.y), (b.vx, b.vy), VISION_BLEND_TAU, VISION_SNAP_DIST)
