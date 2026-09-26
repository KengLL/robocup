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
    FIELD_H,
    FIELD_W,
    NUM_ROBOTS,
    VISION_BALL_COAST_S,
    VISION_BALL_MAX_SPEED,
    VISION_BLEND_TAU,
    VISION_FIELD_MARGIN,
    VISION_FLIP_X,
    VISION_FLIP_Y,
    VISION_FRESH_S,
    VISION_GATE_MARGIN,
    VISION_GATE_RELOCK,
    VISION_REPLAY_GAP_CAP,
    VISION_ROBOT_COAST_S,
    VISION_ROBOT_MAX_SPEED,
    VISION_SNAP_ANGLE,
    VISION_SNAP_DIST,
    VISION_STATS_WINDOW,
)

FRESH, COAST, LOST = "fresh", "coast", "lost"

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
    max_speed: float  # m/s, outlier gate
    coast_s: float
    x: float = 0.0  # real field, meters
    y: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    theta: float = 0.0  # rad
    omega: float = 0.0  # rad/s
    t_meas: float | None = None  # wall-clock capture time of last accepted sighting
    rejects: int = 0  # current streak
    rejected: int = 0  # total
    seen: deque = field(default_factory=lambda: deque(maxlen=VISION_STATS_WINDOW))

    def gate(self, x: float, y: float, t: float) -> bool:
        if self.t_meas is None or t - self.t_meas > self.coast_s:
            return True
        gap = t - self.t_meas
        px, py = self.x + self.vx * gap, self.y + self.vy * gap
        if math.hypot(x - px, y - py) <= self.max_speed * gap + VISION_GATE_MARGIN:
            return True
        self.rejects += 1
        self.rejected += 1
        # A run of rejects means the old track was wrong, not the camera.
        return self.rejects >= VISION_GATE_RELOCK


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

        self.ball = _Track(VISION_BALL_MAX_SPEED, VISION_BALL_COAST_S)
        self.robots = {rid: self._new_robot_track() for rid in self.tag_map.values()}

        self.field_w, self.field_h = 1.2, 0.8  # tracker default, replaced by each doc
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
        return _Track(VISION_ROBOT_MAX_SPEED, VISION_ROBOT_COAST_S)

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
            if not self._warned_aspect:
                a_real, a_sim = self.field_w / self.field_h, FIELD_W / FIELD_H
                if abs(a_real / a_sim - 1.0) > 0.02:
                    print(
                        f"[VisionBridge] WARNING real field {self.field_w}x{self.field_h} "
                        + f"and sim {FIELD_W}x{FIELD_H} differ in aspect; headings will skew"
                    )
                self._warned_aspect = True

            seen_rids: set[int] = set()
            for tag in doc.get("tags") or []:
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
                self._ingest(self.ball, ball, t_cap, heading=False)
            else:
                self.ball.seen.append(False)

    def _ingest(self, tr: _Track, e: dict, t_cap: float, heading: bool) -> None:
        # Only real sightings count. The tracker's own coast is constant-velocity
        # with no friction and has been seen running off the field for 13 s.
        x, y = float(e.get("x", 0.0)), float(e.get("y", 0.0))
        m = VISION_FIELD_MARGIN
        ok = (
            e.get("visible", False)
            and not e.get("lost", False)
            and -m <= x <= self.field_w + m
            and -m <= y <= self.field_h + m
            and tr.gate(x, y, t_cap)
        )
        tr.seen.append(bool(ok))
        if not ok:
            return
        tr.x, tr.y = x, y
        tr.vx, tr.vy = float(e.get("vx", 0.0)), float(e.get("vy", 0.0))
        if heading:
            tr.theta = math.radians(float(e.get("theta_deg", 0.0)))
            tr.omega = math.radians(float(e.get("omega_deg", 0.0)))
        tr.t_meas = t_cap
        tr.rejects = 0

    # ── output (sim thread) ──────────────────────────────────────────────────

    def _to_sim(self, tr: _Track, age: float, mode: str) -> Sighting:
        # Dead-reckon over capture latency plus the time since the last frame.
        dt = age if mode == FRESH else 0.0
        x, y = tr.x + tr.vx * dt, tr.y + tr.vy * dt
        theta = tr.theta + tr.omega * dt
        # TODO(hardware): uniform scale inflates speeds (x7.5 for a 1.2 m table);
        # run the sim at real field size before trusting dynamics.
        sx, sy = FIELD_W / self.field_w, FIELD_H / self.field_h
        x, y, vx, vy = x * sx, y * sy, tr.vx * sx, tr.vy * sy
        omega = tr.omega
        if self.flip_x:
            x, vx, theta, omega = FIELD_W - x, -vx, math.pi - theta, -omega
        if self.flip_y:
            y, vy, theta, omega = FIELD_H - y, -vy, -theta, -omega
        return Sighting(mode, age, x, y, vx, vy, theta, omega)

    def _sighting(self, tr: _Track, now: float) -> Sighting:
        if tr.t_meas is None:
            return Sighting(LOST, math.inf)
        age = max(0.0, now - tr.t_meas)
        if age < VISION_FRESH_S:
            mode = FRESH
        elif age < tr.coast_s:
            mode = COAST
        else:
            mode = LOST
        return self._to_sim(tr, age, mode)

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
    """Fresh: camera drives the body. Coast: physics carries it.
    Lost: robot freezes in place, ball goes back to the sim."""
    for rid, s in snap["robots"].items():
        if s.mode == FRESH:
            world.drive_robot(
                rid, (s.x, s.y), (s.vx, s.vy), s.angle, s.omega,
                VISION_BLEND_TAU, VISION_SNAP_DIST, VISION_SNAP_ANGLE,
            )
        elif s.mode == COAST:
            world.coast_robot(rid)
        else:
            world.freeze_robot(rid)
    b = snap["ball"]
    if b.mode == FRESH:
        world.drive_ball((b.x, b.y), (b.vx, b.vy), VISION_BLEND_TAU, VISION_SNAP_DIST)
