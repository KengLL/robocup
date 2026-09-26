#!/usr/bin/env python3
"""
Visualization Node — single-robot manual control.

Subscribes to world state on VISION_PORT (ZMQ SUB).
Publishes targets          on MANUAL_PORT (ZMQ PUB).

Controls
--------
  Mouse click — send target for robot 0
  1 2 3       — controller: 1=PD inversion  2=MPC  3=Time-optimal (bang-bang)
    Enter       — kick with robot 0 if ball is in front kick zone
"""

from __future__ import annotations
from typing import Any
import json
import math
import os
import random
import sys
import time

import pygame
import zmq

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from config import (  # noqa: E402
    ARRIVAL_THRESH,
    DISPLAY_H,
    DISPLAY_SCALE,
    DISPLAY_W,
    FIELD_H,
    FIELD_W,
    FPS,
    GOAL_Y_MAX,
    GOAL_Y_MIN,
    MANUAL_PORT,
    NUM_HUMAN_CONTROLLERS,
    ROBOT_RADIUS,
    STRATEGY_PORT,
    TEAM_BLUE_SIZE,
    VISION_PORT,
    WHEEL_ANGLES,
)

# ── Palette ───────────────────────────────────────────────────────────────────
C_FIELD = (0, 100, 0)
C_LINE = (255, 255, 255)
C_ROBOT = (30, 144, 255)
C_WHEEL = (200, 0, 0)
C_HUD_BG = (30, 30, 30)
C_GOAL_LEFT = (0, 255, 255)
C_GOAL_RIGHT = (255, 110, 110)

MODES = ["2005_INVERSION", "2005_TIME_OPTIMAL", "MPC", "MANUAL", "CONTROLLER"]
MODE_COLORS = {
    "2005_INVERSION": (0, 255, 128),
    "MPC": (0, 200, 255),
    "2005_TIME_OPTIMAL": (220, 80, 255),
    "MANUAL": (255, 165, 0),
    "CONTROLLER": (255, 105, 180),
}
MODE_KEYS = {pygame.K_1: 0, pygame.K_2: 1, pygame.K_3: 2, pygame.K_4: 3, pygame.K_6: 4}

HUD_MODE_LABELS = {
    "2005_INVERSION": "PD",
    "2005_TIME_OPTIMAL": "TIME",
    "MPC": "MPC",
    "MANUAL": "WASD",
    "CONTROLLER": "PAD",
}

# Strategy toggle
C_STRAT_ON = (0, 255, 0)
C_STRAT_OFF = (255, 60, 60)
C_STALE = (130, 130, 130)  # vision-mirrored object the camera isn't currently seeing
C_ATTACKER = (255, 215, 0)  # gold — outlines the robot currently acting as attacker

# Manual / gamepad settings
MANUAL_MAX_OMEGA = 5.0   # rad/s for full stick deflection
JOYSTICK_DEADZONE = 0.10
# Xbox controller axis indices (adjust if your OS maps them differently):
#   Axis 0 = Left stick X,  Axis 1 = Left stick Y (up = -1)
#   Axis 2 = Right stick X  (right = +1 → clockwise → negative ω)
JS_AXIS_LX, JS_AXIS_LY, JS_AXIS_RX = 0, 1, 2
# Xbox button indices (SDL2 mapping):
#   A=0  B=1  X=2  Y=3  LB=4  RB=5  BACK=6  START=7  LS=8  RS=9
JS_BTN_KICK = 0    # A — single-press
JS_BTN_DRIBBLE = 5 # RB — held

HUD_H = 26
# Top header band carries the timer and half label without overlapping the
# field. Field is drawn between y=HEADER_H and y=HEADER_H+DISPLAY_H.
HEADER_H = 22

# Goal geometry (GOAL_Y_MIN / GOAL_Y_MAX) is imported from config.py.

PIXEL_GLYPHS = {
    "0": ["111", "101", "101", "101", "111"],
    "1": ["010", "110", "010", "010", "111"],
    "2": ["111", "001", "111", "100", "111"],
    "3": ["111", "001", "111", "001", "111"],
    "4": ["101", "101", "111", "001", "001"],
    "5": ["111", "100", "111", "001", "111"],
    "6": ["111", "100", "111", "101", "111"],
    "7": ["111", "001", "001", "010", "010"],
    "8": ["111", "101", "111", "101", "111"],
    "9": ["111", "101", "111", "001", "111"],
    "-": ["000", "000", "111", "000", "000"],
    "A": ["010", "101", "111", "101", "101"],
    "P": ["111", "101", "111", "100", "100"],
    "D": ["110", "101", "101", "101", "110"],
    "T": ["111", "010", "010", "010", "010"],
    "H": ["101", "101", "111", "101", "101"],
    "W": ["101", "101", "101", "111", "010"],
    "I": ["111", "010", "010", "010", "111"],
    "M": ["101", "111", "111", "101", "101"],
    "U": ["101", "101", "101", "101", "111"],
    "E": ["111", "100", "111", "100", "111"],
    "C": ["111", "100", "100", "100", "111"],
    "L": ["100", "100", "100", "100", "111"],
    "R": ["110", "101", "110", "101", "101"],
    "S": ["111", "100", "111", "001", "111"],
    "O": ["111", "101", "101", "101", "111"],
    "N": ["101", "111", "111", "111", "101"],
    "F": ["111", "100", "111", "100", "100"],
    "G": ["111", "100", "101", "101", "111"],
    "K": ["101", "101", "110", "101", "101"],
    "Y": ["101", "101", "010", "010", "010"],
    "B": ["110", "101", "110", "101", "110"]
}


# ── helpers ───────────────────────────────────────────────────────────────────


def w2s(x: float, y: float) -> tuple[int, int]:
    """World metres (y-up) → screen pixels (y-down). Field is drawn below
    the header band, so screen-y starts at HEADER_H."""
    return int(x * DISPLAY_SCALE), int(HEADER_H + DISPLAY_H - y * DISPLAY_SCALE)


def s2w(px: int, py: int) -> tuple[float, float]:
    """Screen pixels → world metres (clamped to field)."""
    wx = max(0.0, min(FIELD_W, px / DISPLAY_SCALE))
    wy = max(0.0, min(FIELD_H, (HEADER_H + DISPLAY_H - py) / DISPLAY_SCALE))
    return wx, wy


# ── drawing ───────────────────────────────────────────────────────────────────


def draw_field(surf: pygame.Surface) -> None:
    _ = surf.fill(C_FIELD)
    _ = pygame.draw.rect(surf, C_LINE, (0, HEADER_H, DISPLAY_W, DISPLAY_H), 3)
    _ = pygame.draw.line(
        surf, C_LINE, (DISPLAY_W // 2, HEADER_H), (DISPLAY_W // 2, HEADER_H + DISPLAY_H), 2
    )
    _ = pygame.draw.circle(
        surf, C_LINE, (DISPLAY_W // 2, HEADER_H + DISPLAY_H // 2), int(0.5 * DISPLAY_SCALE), 2
    )

    # Goal-post markers overlaid at the side edges.
    _, gy0 = w2s(0.0, GOAL_Y_MAX)
    _, gy1 = w2s(0.0, GOAL_Y_MIN)
    # Extra white underlay on blue post for stronger contrast against grass.
    _ = pygame.draw.line(surf, (255, 255, 255), (0, gy0), (0, gy1), 8)
    _ = pygame.draw.line(surf, C_GOAL_LEFT, (0, gy0), (0, gy1), 5)
    _ = pygame.draw.line(surf, C_GOAL_RIGHT, (DISPLAY_W - 1, gy0), (DISPLAY_W - 1, gy1), 6)


def draw_robot(
    surf: pygame.Surface,
    x: float,
    y: float,
    angle: float,
    color: tuple[int, int, int],
    is_attacker: bool = False,
) -> None:
    cx, cy = w2s(x, y)
    r = max(4, int(ROBOT_RADIUS * DISPLAY_SCALE))
    _ = pygame.draw.circle(surf, color, (cx, cy), r)
    outline = C_ATTACKER if is_attacker else C_LINE
    width = 3 if is_attacker else 2
    _ = pygame.draw.circle(surf, outline, (cx, cy), r, width)
    # Heading arrow
    ex = int(cx + math.cos(angle) * r * 1.4)
    ey = int(cy - math.sin(angle) * r * 1.4)
    _ = pygame.draw.line(surf, outline, (cx, cy), (ex, ey), 3)
    # Wheel dots
    for alpha in WHEEL_ANGLES:
        wx = int(cx + math.cos(angle + alpha) * r)
        wy = int(cy - math.sin(angle + alpha) * r)
        _ = pygame.draw.circle(surf, C_WHEEL, (wx, wy), 4)


def draw_dotted_line(
    surf: pygame.Surface,
    x0: float,
    y0: float,
    x1: float,
    y1: float,
    color: tuple[int, int, int],
    spacing: int = 12,
) -> None:
    sx0, sy0 = w2s(x0, y0)
    sx1, sy1 = w2s(x1, y1)
    dx, dy = sx1 - sx0, sy1 - sy0
    length = math.hypot(dx, dy)
    if length < 1:
        return
    steps = max(1, int(length / spacing))
    for i in range(steps + 1):
        t = i / steps
        px = int(sx0 + dx * t)
        py = int(sy0 + dy * t)
        _ = pygame.draw.circle(surf, color, (px, py), 2)


def draw_pin(surf: pygame.Surface, x: float, y: float, color: tuple[int, int, int]) -> None:
    """Map-pin icon: filled circle head + vertical stem."""
    cx, cy = w2s(x, y)
    stem_top = cy - 22
    head_r = 7
    # Stem
    _ = pygame.draw.line(surf, color, (cx, cy), (cx, stem_top + head_r), 2)
    # Head
    _ = pygame.draw.circle(surf, color, (cx, stem_top), head_r)
    _ = pygame.draw.circle(surf, (255, 255, 255), (cx, stem_top), head_r, 1)


def draw_pixel_text(
    surf: pygame.Surface,
    text: str,
    x: int,
    y: int,
    color: tuple[int, int, int],
    pixel: int = 2,
    spacing: int = 1,
) -> None:
    cursor_x = x
    for ch in text:
        glyph = PIXEL_GLYPHS.get(ch)
        if glyph is None:
            cursor_x += (3 * pixel) + spacing + pixel
            continue
        for row, bits in enumerate(glyph):
            for col, bit in enumerate(bits):
                if bit == "1":
                    _ = pygame.draw.rect(
                        surf,
                        color,
                        (cursor_x + col * pixel, y + row * pixel, pixel, pixel),
                    )
        cursor_x += (3 * pixel) + spacing + pixel


def draw_vision_stats(surf: pygame.Surface, st: dict[str, Any]) -> None:
    drops = "  ".join(
        f"{'ball' if k == 'ball' else 'r' + k} {'-' if v is None else f'{v:.0f}%'}"
        for k, v in st.get("drop_pct", {}).items()
    )
    link = "UP" if st.get("link_up") else "DOWN"
    # cam drops: frames the tracker was too slow for; link drops: lost between tracker and sim
    cam = st.get("tracker", {}).get("capture_dropped")
    text = (
        f"VISION {st.get('source', '?')} {link}  {st.get('fps', 0):.0f} fps  "
        f"lat {st.get('latency_ms', 0):.0f} ms  "
        + ("" if cam is None else f"cam drops {cam}  ")
        + f"link drops {st.get('transport_drops', 0)}  "
        f"rejects {st.get('rejects', 0)}  unseen {drops}"
    )
    font = _load_mono_font(13)
    img = font.render(text, True, (255, 80, 80) if link == "DOWN" else C_LINE)
    y = HEADER_H + DISPLAY_H - img.get_height() - 6
    bg = pygame.Surface((img.get_width() + 8, img.get_height() + 4), pygame.SRCALPHA)
    bg.fill((0, 0, 0, 150))
    surf.blit(bg, (6, y - 2))
    surf.blit(img, (10, y))


def draw_hud(
    surf: pygame.Surface,
    mode_idx: int,
    strategy_on: bool = False,
    score_blue: int = 0,
    score_red: int = 0,
    game_info: dict[str, Any] | None = None,
    rl_kick_on: bool = False,
) -> None:
    """Bottom status bar — mode selector blocks + centered score + strategy toggle."""
    y0 = HEADER_H + DISPLAY_H
    pad = 5
    _ = pygame.draw.rect(surf, C_HUD_BG, (0, y0, DISPLAY_W, HUD_H))

    # timer and half display above HUD
    if game_info is not None:
        half = game_info.get("half", 1)
        time_remaining = game_info.get("time_remaining", 300.0)
        half_over = game_info.get("half_over", False)
        total_seconds = int(time_remaining)

        # draw small top bar
        _ = pygame.draw.rect(surf, (20, 20, 20), (0, 0, DISPLAY_W, 22))

        # half label on left
        if half_over:
            half_label = "FULLTIME"
        elif half == 1:
            half_label = "HALF1"
        elif half == 2:
            half_label = "HALF2"
        elif half == 3:
            half_label = "OT1"
        elif half == 4:
            half_label = "OT2"
        else:
            half_label = "HALF1"
        draw_pixel_text(surf, half_label, 6, 6, (180, 180, 180), pixel=2, spacing=1)

        # timer in center — goes red under 30s
        timer_str = str(total_seconds)
        timer_color = (255, 60, 60) if time_remaining < 30 else (255, 255, 255)
        pixel = 2
        char_step = (3 * pixel) + 1 + pixel
        total_w = len(timer_str) * char_step - pixel
        tx = (DISPLAY_W - total_w) // 2
        draw_pixel_text(surf, timer_str, tx, 6, timer_color, pixel=pixel, spacing=1)

    block_w = 40
    x = pad
    active_mode = MODES[mode_idx]

    for mode in MODES:
        active = mode == active_mode
        color = (
            MODE_COLORS[mode] if active else tuple(v // 4 for v in MODE_COLORS[mode])
        )
        _ = pygame.draw.rect(
            surf, color, (x, y0 + pad, block_w, HUD_H - pad * 2), border_radius=3
        )
        if active:
            _ = pygame.draw.rect(
                surf,
                (255, 255, 255),
                (x, y0 + pad, block_w, HUD_H - pad * 2),
                1,
                border_radius=3,
            )

        label = HUD_MODE_LABELS[mode]
        pixel = 2
        glyph_w = 3 * pixel
        glyph_h = 5 * pixel
        char_step = glyph_w + 1 + pixel
        total_w = len(label) * char_step - pixel
        tx = x + (block_w - total_w) // 2
        ty = y0 + (HUD_H - glyph_h) // 2
        draw_pixel_text(
            surf,
            label,
            tx,
            ty,
            (255, 255, 255) if active else (80, 80, 80),
            pixel=pixel,
            spacing=1,
        )

        x += block_w + pad

    # Center scoreboard
    score_text = f"{score_blue}-{score_red}"
    pixel = 3
    glyph_w = 3 * pixel
    glyph_h = 5 * pixel
    char_step = glyph_w + 1 + pixel
    total_w = len(score_text) * char_step - pixel
    score_x = (DISPLAY_W - total_w) // 2
    score_y = y0 + (HUD_H - glyph_h) // 2
    draw_pixel_text(
        surf,
        score_text,
        score_x,
        score_y,
        (245, 245, 245),
        pixel=pixel,
        spacing=1,
    )

    # Strategy + RL-kick toggle indicators (right side, stacked).
    pixel = 2
    glyph_w = 3 * pixel
    char_step = glyph_w + 1 + pixel
    strat_label = "AUTO ON" if strategy_on else "AUTO OFF"
    strat_color = C_STRAT_ON if strategy_on else C_STRAT_OFF
    strat_w = len(strat_label) * char_step - pixel
    sx = DISPLAY_W - strat_w - pad
    sy = y0 + (HUD_H - 5 * pixel) // 2
    draw_pixel_text(surf, strat_label, sx, sy, strat_color, pixel=pixel, spacing=1)

    rl_label = "RL ON" if rl_kick_on else "RL OFF"
    rl_color = C_STRAT_ON if rl_kick_on else C_STRAT_OFF
    rl_w = len(rl_label) * char_step - pixel
    rx = sx - rl_w - pad * 2
    draw_pixel_text(surf, rl_label, rx, sy, rl_color, pixel=pixel, spacing=1)


def _spawn_confetti(particles: list[dict[str, Any]], team: str) -> None:
    cx, cy = w2s(FIELD_W / 2.0, FIELD_H / 2.0)
    palette = (
        [(30, 144, 255), (130, 205, 255), (255, 255, 255)]
        if team == "blue"
        else [(255, 80, 80), (255, 170, 120), (255, 255, 255)]
    )
    for _ in range(180):
        angle = random.uniform(0.0, math.tau)
        speed = random.uniform(130.0, 460.0)
        particles.append(
            {
                "x": float(cx),
                "y": float(cy),
                "vx": math.cos(angle) * speed,
                "vy": math.sin(angle) * speed,
                "life": random.uniform(0.9, 1.8),
                "max_life": 1.8,
                "size": random.randint(2, 5),
                "color": random.choice(palette),
            }
        )


def _update_confetti(particles: list[dict[str, Any]], dt: float) -> None:
    gravity = 700.0
    alive = []
    for p in particles:
        p["life"] -= dt
        if p["life"] <= 0.0:
            continue
        p["vy"] += gravity * dt
        p["x"] += p["vx"] * dt
        p["y"] += p["vy"] * dt
        alive.append(p)
    particles[:] = alive


def draw_confetti(surf: pygame.Surface, particles: list[dict[str, Any]]) -> None:
    for p in particles:
        if p["x"] < 0 or p["x"] >= DISPLAY_W or p["y"] < HEADER_H or p["y"] >= HEADER_H + DISPLAY_H:
            continue
        scale = max(0.35, p["life"] / p["max_life"])
        radius = max(1, int(p["size"] * scale))
        _ = pygame.draw.circle(surf, p["color"], (int(p["x"]), int(p["y"])), radius)


def draw_goal_flash(
    surf: pygame.Surface,
    team: str,
    score_text: str,
    remaining: float,
) -> None:
    if remaining <= 0.0:
        return
    team_color = (100, 190, 255) if team == "blue" else (255, 110, 110)
    alpha = int(max(0, min(170, 170 * (remaining / 2.0))))
    tint = pygame.Surface((DISPLAY_W, DISPLAY_H), pygame.SRCALPHA)
    _ = tint.fill((team_color[0], team_color[1], team_color[2], alpha))
    _ = surf.blit(tint, (0, 0))

    goal_text = "GOAL"
    goal_px = 14
    goal_step = (3 * goal_px) + 1 + goal_px
    goal_w = len(goal_text) * goal_step - goal_px
    goal_h = 5 * goal_px
    gx = (DISPLAY_W - goal_w) // 2
    gy = (DISPLAY_H // 2) - 130
    draw_pixel_text(surf, goal_text, gx, gy, (255, 255, 255), pixel=goal_px, spacing=1)

    score_px = 22
    score_step = (3 * score_px) + 1 + score_px
    score_w = len(score_text) * score_step - score_px
    sx = (DISPLAY_W - score_w) // 2
    sy = gy + goal_h + 36
    draw_pixel_text(surf, score_text, sx, sy, team_color, pixel=score_px, spacing=1)

def draw_phase_popup(
    surf: pygame.Surface,
    text: str,
    remaining: float,
    max_duration: float = 3.0,
) -> None:
    """Big centered popup for phase changes like HALF TIME, OVERTIME etc."""
    if remaining <= 0.0:
        return

    # fade out in last 0.5 seconds
    alpha = int(min(200, 200 * (remaining / 0.5))) if remaining < 0.5 else 200
    tint = pygame.Surface((DISPLAY_W, DISPLAY_H), pygame.SRCALPHA)
    _ = tint.fill((0, 0, 0, alpha // 2))
    _ = surf.blit(tint, (0, 0))

    # big text
    pixel = 10
    char_step = (3 * pixel) + 1 + pixel
    # split into two lines if needed
    words = text.split()
    lines = []
    if len(words) == 1:
        lines = [text]
    else:
        lines = [words[0], " ".join(words[1:])]

    total_h = len(lines) * (5 * pixel + 10)
    start_y = DISPLAY_H // 2 - total_h // 2

    for i, line in enumerate(lines):
        clean = line.replace(" ", "")
        total_w = len(clean) * char_step - pixel
        tx = (DISPLAY_W - total_w) // 2
        ty = start_y + i * (5 * pixel + 10)
        draw_pixel_text(surf, clean, tx, ty, (255, 255, 255), pixel=pixel, spacing=1)

def draw_winner_screen(
    surf: pygame.Surface,
    winner: str,
    score_blue: int,
    score_red: int,
) -> None:
    """Full screen winner display shown at game end."""
    # dark overlay
    overlay = pygame.Surface((DISPLAY_W, DISPLAY_H), pygame.SRCALPHA)
    _ = overlay.fill((0, 0, 0, 220))
    _ = surf.blit(overlay, (0, 0))

    if winner == "blue":
        win_color = (30, 144, 255)
        win_text = "BLUE WINS"
    elif winner == "red":
        win_color = (255, 80, 80)
        win_text = "RED WINS"
    else:
        win_color = (200, 200, 200)
        win_text = "DRAW"

    # winner text
    pixel = 12
    char_step = (3 * pixel) + 1 + pixel
    clean = win_text.replace(" ", "")
    total_w = len(clean) * char_step - pixel
    tx = (DISPLAY_W - total_w) // 2
    draw_pixel_text(surf, clean, tx, DISPLAY_H // 2 - 80, win_color, pixel=pixel, spacing=1)

    # score
    score_str = f"{score_blue}-{score_red}"
    pixel = 16
    char_step = (3 * pixel) + 1 + pixel
    total_w = len(score_str) * char_step - pixel
    sx = (DISPLAY_W - total_w) // 2
    draw_pixel_text(surf, score_str, sx, DISPLAY_H // 2, (255, 255, 255), pixel=pixel, spacing=1)

    # subtext
    sub = "FULLTIME"
    pixel = 4
    char_step = (3 * pixel) + 1 + pixel
    total_w = len(sub) * char_step - pixel
    subx = (DISPLAY_W - total_w) // 2
    draw_pixel_text(surf, sub, subx, DISPLAY_H // 2 + 90, (160, 160, 160), pixel=pixel, spacing=1)

# Cache mono-font lookups so we don't re-stat font files on every redraw.
_MONO_FONT_CACHE: dict[tuple[int, bool], pygame.font.Font] = {}
# Print which path was used on the first successful (or failed) load.
_MONO_FONT_LOGGED = False


def _load_mono_font(size: int, bold: bool = False) -> pygame.font.Font:
    """Load a monospace TTF directly, bypassing pygame.font.SysFont().

    SysFont scans the Windows font registry on first call and crashes
    (TypeError on splitext) when any HKLM\\...\\Fonts entry is a DWORD
    instead of a path string. Loading a .ttf path skips that scan."""
    global _MONO_FONT_LOGGED
    key = (size, bold)
    cached = _MONO_FONT_CACHE.get(key)
    if cached is not None:
        return cached
    candidates = (
        r"C:\Windows\Fonts\consola.ttf",
        r"C:\Windows\Fonts\cour.ttf",
        "/System/Library/Fonts/Menlo.ttc",
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
    )
    font: pygame.font.Font | None = None
    chosen: str | None = None
    attempts: list[tuple[str, str]] = []
    for path in candidates:
        try:
            font = pygame.font.Font(path, size)
            chosen = path
            break
        except (FileNotFoundError, OSError) as e:
            attempts.append((path, f"{type(e).__name__}: {e}"))
            continue
    if font is None:
        if not _MONO_FONT_LOGGED:
            print("[VizNode] WARNING: no monospace font found, falling back to pygame default (not monospace)")
            for path, err in attempts:
                print(f"[VizNode]   tried {path} -> {err}")
            _MONO_FONT_LOGGED = True
        font = pygame.font.Font(None, size)  # bundled default — not monospace
    else:
        if not _MONO_FONT_LOGGED:
            print(f"[VizNode] mono font loaded: {chosen}")
            _MONO_FONT_LOGGED = True
    if bold:
        font.set_bold(True)
    _MONO_FONT_CACHE[key] = font
    return font


def draw_controls_screen(surf: pygame.Surface, close_rect: pygame.Rect) -> None:
    """Full screen controls overlay shown at startup."""
    overlay = pygame.Surface((DISPLAY_W, HEADER_H + DISPLAY_H + HUD_H), pygame.SRCALPHA)
    overlay.fill((0, 0, 0, 230))
    surf.blit(overlay, (0, 0))

    # Title
    title = "CONTROLS"
    pixel = 8
    char_step = (3 * pixel) + 1 + pixel
    total_w = len(title) * char_step - pixel
    draw_pixel_text(surf, title, (DISPLAY_W - total_w) // 2, 30, (255, 215, 0), pixel=pixel, spacing=1)

    # Controls list
    controls = [
        ("1 2 3",    "PD / TIME-OPTIMAL / MPC controller"),
        ("4",        "WASD mode  (WASD move  QE rotate, robot 0)"),
        ("6",        "PAD mode   (gamepad sticks drive blue robots)"),
        ("5",        "Toggle autonomous strategy AI"),
        ("R",        "Toggle RL kick policy"),
        ("K",        "Kick (robot 0)"),
        ("0",        "Toggle AI dribbler"),
        ("9",        "Toggle ball-stuck auto-reset"),
        ("CLICK",    "Send robot 0 to clicked position"),
        ("STICKS",   "Left = move  Right = rotate (per controller)"),
        ("A / RB",   "Kick / hold to dribble (per controller)"),
    ]

    game_rules = [
        ("HALVES",   "2 x 300 seconds"),
        ("OVERTIME", "If scores equal after 90 min"),
        ("10 GOALS", "Game ends on 10-goal lead"),
        ("SCORING",  "Ball must enter goal mouth"),
    ]

    # Draw two columns
    col_x = [40, DISPLAY_W // 2 + 20]
    headers = ["KEYBOARD", "GAME RULES"]
    datasets = [controls, game_rules]

    for col, (header, data) in enumerate(zip(headers, datasets)):
        hx = col_x[col]
        hy = 80
        # section header
        pixel = 3
        draw_pixel_text(surf, header, hx, hy, (100, 200, 255), pixel=pixel, spacing=1)

        # use pygame font for the description text — pixel glyphs don't have lowercase
        font_small = _load_mono_font(13)
        font_key   = _load_mono_font(13, bold=True)

        y = hy + 22
        for key, desc in data:
            key_surf = font_key.render(f"[{key}]", True, (255, 215, 0))
            desc_surf = font_small.render(desc, True, (200, 200, 200))
            surf.blit(key_surf,  (hx, y))
            surf.blit(desc_surf, (hx + key_surf.get_width() + 8, y))
            y += 22

    # X close button
    pygame.draw.rect(surf, (180, 40, 40), close_rect, border_radius=6)
    pygame.draw.rect(surf, (255, 255, 255), close_rect, 2, border_radius=6)
    font_x = _load_mono_font(18, bold=True)
    x_surf = font_x.render("X  CLOSE", True, (255, 255, 255))
    surf.blit(x_surf, (
        close_rect.x + (close_rect.width  - x_surf.get_width())  // 2,
        close_rect.y + (close_rect.height - x_surf.get_height()) // 2,
    ))

    # small hint at bottom
    font_hint = _load_mono_font(11)
    hint = font_hint.render("press any key or click X to dismiss", True, (120, 120, 120))
    surf.blit(hint, ((DISPLAY_W - hint.get_width()) // 2, HEADER_H + DISPLAY_H + HUD_H - 20))

# ── main ──────────────────────────────────────────────────────────────────────

def _apply_deadzone(v: float, dz: float) -> float:
    return 0.0 if abs(v) < dz else v



def main() -> None:
    _ = pygame.init()
    pygame.mixer.init()


    pygame.joystick.init()
    # Each connected controller drives blue robot at the same index. Cap at
    # both NUM_HUMAN_CONTROLLERS (config) and what's actually plugged in.
    n_avail = pygame.joystick.get_count()
    n_human = min(n_avail, NUM_HUMAN_CONTROLLERS, TEAM_BLUE_SIZE)
    joysticks: list[pygame.joystick.Joystick] = []
    for ji in range(n_human):
        js = pygame.joystick.Joystick(ji)
        joysticks.append(js)
        print(f"[VizNode] Controller {ji} → blue robot {ji}: {js.get_name()}")
    if NUM_HUMAN_CONTROLLERS > 0 and not joysticks:
        print(f"[VizNode] no controllers plugged in (config wants {NUM_HUMAN_CONTROLLERS}); blue is fully AI")
    elif n_avail > n_human:
        print(f"[VizNode] {n_avail - n_human} extra controller(s) ignored (NUM_HUMAN_CONTROLLERS={NUM_HUMAN_CONTROLLERS})")
    human_blue_ids = list(range(len(joysticks)))
    # Per-controller previous A-button state for rising-edge kick detection.
    prev_kick_btn: list[bool] = [False] * len(joysticks)

    screen = pygame.display.set_mode((DISPLAY_W, HEADER_H + DISPLAY_H + HUD_H))
    pygame.display.set_caption(
        "RoboCup — click to move  |  1/2/3: PD/TIME/MPC  |  4: WASD  |  6: PAD  |  5: Strategy  |  R: RL Kick  |  Enter: Kick  |  9: Toggle Ball Stuck Reset | 0: Toggle Dribble"
    )
    clock = pygame.time.Clock()

    # ── Sounds ─────────────────────────────
    SND_GOAL = pygame.mixer.Sound("sounds/goal.wav")
    SND_CHEER = pygame.mixer.Sound("sounds/cheer.wav")
    SND_WHISTLE = pygame.mixer.Sound("sounds/whistle.wav")
    SND_WINNER = pygame.mixer.Sound("sounds/winner.wav")
    SND_COUNTDOWN = pygame.mixer.Sound("sounds/countdown.wav")

    # volume tuning
    SND_GOAL.set_volume(0.2)
    SND_CHEER.set_volume(0.6)
    SND_WHISTLE.set_volume(0.7)

    ctx = zmq.Context()

    vision_sub = ctx.socket(zmq.SUB)
    _ = vision_sub.connect(f"tcp://localhost:{VISION_PORT}")
    vision_sub.setsockopt_string(zmq.SUBSCRIBE, "")
    vision_sub.setsockopt(zmq.RCVTIMEO, 0)

    # Strategy stream — used as the source of truth for the attacker outline
    # so the gold ring matches the robot strategy_node actually told to kick.
    strategy_sub = ctx.socket(zmq.SUB)
    _ = strategy_sub.connect(f"tcp://localhost:{STRATEGY_PORT}")
    strategy_sub.setsockopt_string(zmq.SUBSCRIBE, "")
    strategy_sub.setsockopt(zmq.RCVTIMEO, 0)

    manual_pub = ctx.socket(zmq.PUB)
    _ = manual_pub.bind(f"tcp://*:{MANUAL_PORT}")

    # ZMQ PUB/SUB has a slow-joiner window; sleep so the launcher's already-up
    # subscribers have time to attach before we send the human_blue_ids hello.
    # Resent below on every strategy toggle as a safety net.
    time.sleep(0.3)
    _ = manual_pub.send_string(json.dumps({"human_blue_ids": human_blue_ids}))

    world_state: dict[str, Any] | None = None
    attacker_rids: set[str] = set()
    score_blue = 0
    score_red = 0
    game_info = None
    phase_popup_text = ""
    phase_popup_until = 0.0
    last_phase_seen = "FIRST HALF"
    game_winner = None  # "blue", "red", "draw", or None
    last_countdown_second = None
    last_goal_seq_seen = -1
    goal_flash_team = "blue"
    goal_flash_until = 0.0
    goal_flash_score_text = "0-0"
    confetti_particles: list[dict[str, Any]] = []

    mode_idx = 0
    strategy_enabled = False
    rl_kick_enabled = False
    dribble_on = False
    ball_stuck_on = False
    ball_stuck_popup_until = 0.0
    last_ball_stuck_seq = -1

    show_controls = True
    controls_close_rect = pygame.Rect(DISPLAY_W // 2 - 80, HEADER_H + DISPLAY_H - 60, 160, 40)

    # Overlay state: cleared on arrival
    target_pin: tuple[float, float] | None = None
    path_start: tuple[float, float] | None = None

    print(
        "[VizNode] Click field to move robot  |  1=PD  2=TIME  3=MPC  "
        + "4=WASD  6=PAD  5=Toggle Strategy  R=Toggle RL Kick  Enter=Kick"
    )

    running = True
    while running:
        dt_s = 1.0 / FPS
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if show_controls:
                if event.type == pygame.KEYDOWN:
                    show_controls = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if controls_close_rect.collidepoint(event.pos):
                        show_controls = False
                continue  # block all other input while screen is up

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_5:
                    strategy_enabled = not strategy_enabled
                    # Re-publish human_blue_ids alongside the toggle so a
                    # late-starting strategy_node always knows the AI subset.
                    _ = manual_pub.send_string(json.dumps(
                        {"strategy_enabled": strategy_enabled,
                         "human_blue_ids": human_blue_ids}
                    ))
                    state_str = "ON" if strategy_enabled else "OFF"
                    print(f"[VizNode] Strategy → {state_str}")
                elif event.key == pygame.K_r:
                    rl_kick_enabled = not rl_kick_enabled
                    _ = manual_pub.send_string(json.dumps(
                        {"rl_kick_enabled": rl_kick_enabled}
                    ))
                    state_str = "ON" if rl_kick_enabled else "OFF"
                    print(f"[VizNode] RL Kick → {state_str}")
                elif event.key in (pygame.K_k, pygame.K_RETURN, pygame.K_KP_ENTER):
                    _ = manual_pub.send_string(json.dumps({"kick": {"0": True}}))
                    print("[VizNode] Kick requested for robot 0")
                elif event.key == pygame.K_0:
                    dribble_on = not dribble_on
                    manual_pub.send_string(json.dumps({"dribble_attacker": dribble_on}))
                    print(f"[VizNode] Dribble (attacker) → {'ON' if dribble_on else 'OFF'}")
                elif event.key == pygame.K_9:
                    ball_stuck_on = not ball_stuck_on
                    manual_pub.send_string(json.dumps({"type": "ball_stuck_toggle"}))
                    print(f"[VizNode] Ball Stuck Reset → {'ON' if ball_stuck_on else 'OFF'}")
                elif event.key in MODE_KEYS:
                    mode_idx = MODE_KEYS[event.key]
                    print(f"[VizNode] Mode → {MODES[mode_idx]}")

            elif event.type == pygame.MOUSEBUTTONDOWN:
                if MODES[mode_idx] not in ("MANUAL", "CONTROLLER"):
                    mx, my = event.pos
                    if HEADER_H <= my < HEADER_H + DISPLAY_H:  # ignore header + HUD
                        wx, wy = s2w(mx, my)
                        mode = MODES[mode_idx]
                        # Snapshot robot position as path start
                        if world_state and "0" in world_state.get("robots", {}):
                            r = world_state["robots"]["0"]
                            path_start = (r["x"], r["y"])
                        else:
                            path_start = None
                        target_pin = (wx, wy)
                        _ = manual_pub.send_string(
                            json.dumps({"targets": {"0": {"x": wx, "y": wy, "mode": mode}}})
                        )
                        print(f"[VizNode] Target → ({wx:.2f}, {wy:.2f})  [{mode}]")

        # ── CONTROLLER mode: each connected gamepad drives blue robot at the
        # same index. strategy_node leaves these robot ids alone (per the
        # human_blue_ids hello), so the controller is the sole source here.
        if joysticks and MODES[mode_idx] == "CONTROLLER":
            direct_msg: dict[str, dict[str, float]] = {}
            kick_msg: dict[str, bool] = {}
            dribble_msg: dict[str, bool] = {}
            for ji, js in enumerate(joysticks):
                rid_str = str(ji)
                lx = _apply_deadzone(js.get_axis(JS_AXIS_LX), JOYSTICK_DEADZONE)
                ly = _apply_deadzone(js.get_axis(JS_AXIS_LY), JOYSTICK_DEADZONE)
                rx = _apply_deadzone(js.get_axis(JS_AXIS_RX), JOYSTICK_DEADZONE)
                direct_msg[rid_str] = {
                    "vx": lx,
                    "vy": -ly,                    # SDL Y axis is inverted (up = -1)
                    "w": -rx * MANUAL_MAX_OMEGA,  # right-stick right = CW = -ω
                }
                # A: rising-edge kick (one shot per press; sending kick=True
                # every frame would trigger a kick on every tick).
                kick = bool(js.get_button(JS_BTN_KICK))
                if kick and not prev_kick_btn[ji]:
                    kick_msg[rid_str] = True
                prev_kick_btn[ji] = kick
                # RB: held dribble — publish the level every frame so a dropped
                # ZMQ message can't latch the dribble on or off.
                dribble_msg[rid_str] = bool(js.get_button(JS_BTN_DRIBBLE))

            payload: dict[str, Any] = {"direct": direct_msg}
            if kick_msg:
                payload["kick"] = kick_msg
            if dribble_msg:
                payload["dribble"] = dribble_msg
            _ = manual_pub.send_string(json.dumps(payload))
            target_pin = None
            path_start = None

        # ── MANUAL mode: WASD moves / QE rotates blue robot 0. Mutually
        # exclusive with CONTROLLER mode by mode-gating, so the two never
        # publish "direct" for the same robot id on the same tick.
        if MODES[mode_idx] == "MANUAL":
            keys = pygame.key.get_pressed()
            vx = (1.0 if keys[pygame.K_d] else 0.0) - (1.0 if keys[pygame.K_a] else 0.0)
            vy = (1.0 if keys[pygame.K_w] else 0.0) - (1.0 if keys[pygame.K_s] else 0.0)
            w  = ((1.0 if keys[pygame.K_q] else 0.0)
                  - (1.0 if keys[pygame.K_e] else 0.0)) * MANUAL_MAX_OMEGA
            _ = manual_pub.send_string(json.dumps(
                {"direct": {"0": {"vx": vx, "vy": vy, "w": w}}}
            ))
            target_pin = None
            path_start = None

        # Drain vision (keep latest frame)
        while True:
            try:
                world_state = json.loads(vision_sub.recv_string())
            except zmq.Again:
                break

        # Drain strategy (keep latest attacker rids).
        while True:
            try:
                strat_msg = json.loads(strategy_sub.recv_string())
                attacker_rids = {str(rid) for rid in strat_msg.get("attackers", [])}
            except zmq.Again:
                break

        if world_state:
            score = world_state.get("score", {})
            score_blue = int(score.get("blue", score_blue))
            score_red = int(score.get("red", score_red))
            game_info = world_state.get("game", None)
            if game_info is not None:

                time_remaining = game_info.get("time_remaining", 300)
                half_over = game_info.get("half_over", False)

                # countdown beeps for last 10 seconds
                if not half_over and time_remaining <= 10.0:
                    current_second = int(time_remaining)
                    if current_second != last_countdown_second and current_second >= 0:
                        last_countdown_second = current_second
                        if current_second > 0:
                            SND_COUNTDOWN.play()
                        else:
                            # zero — play whistle
                            SND_WHISTLE.play()

                # reset countdown tracker when a new half starts
                if time_remaining > 10.0:
                    last_countdown_second = None

                new_phase = game_info.get("phase", "FIRST HALF")
                if new_phase != last_phase_seen:
                    last_phase_seen = new_phase
                    phase_popup_text = new_phase
                    phase_popup_until = time.monotonic() + 3.0
                    print(f"[VizNode] Phase → {new_phase}")
                game_winner = game_info.get("winner", None)
                ball_stuck_seq = game_info.get("ball_stuck_seq", 0)
                if ball_stuck_seq != last_ball_stuck_seq and last_ball_stuck_seq != -1:
                    ball_stuck_popup_until = time.monotonic() + 3.0
                    print("[VizNode] Ball stuck — reset")
                last_ball_stuck_seq = ball_stuck_seq
            last_goal = world_state.get("last_goal")

            if isinstance(last_goal, dict):
                seq = int(last_goal.get("seq", -1))
                if seq > last_goal_seq_seen:
                    SND_GOAL.play()
                    SND_CHEER.play()
                    last_goal_seq_seen = seq
                    scored_post_team = str(last_goal.get("team", "blue"))
                    # Reverse effect color relative to post scored on:
                    goal_flash_team = "blue" if scored_post_team == "red" else "red"
                    gscore = last_goal.get("score", {})
                    goal_flash_score_text = f"{int(gscore.get('blue', score_blue))}-{int(gscore.get('red', score_red))}"
                    goal_flash_until = time.monotonic() + 2.0
                    _spawn_confetti(confetti_particles, goal_flash_team)
                    print(
                        f"[VizNode] GOAL on {scored_post_team.upper()} post  effect {goal_flash_team.upper()}  score {goal_flash_score_text}"
                    )

        _update_confetti(confetti_particles, dt_s)

        # Auto-clear overlay when robot arrives
        if target_pin and world_state:
            r = world_state["robots"].get("0")
            if r:
                dist = math.hypot(r["x"] - target_pin[0], r["y"] - target_pin[1])
                if dist < ARRIVAL_THRESH:
                    target_pin = None
                    path_start = None

        # ── Draw ──────────────────────────────────────────────────────────────
        draw_field(screen)

        # Dotted path + pin (drawn before robot so robot renders on top)
        if target_pin:
            mode_color = MODE_COLORS[MODES[mode_idx]]
            if path_start:
                draw_dotted_line(
                    screen,
                    path_start[0],
                    path_start[1],
                    target_pin[0],
                    target_pin[1],
                    mode_color,
                )
            draw_pin(screen, target_pin[0], target_pin[1], mode_color)

        if world_state:
            robots = world_state.get("robots", {})
            b = world_state.get("ball")

            # attacker_rids comes from strategy_node's published "attackers"
            # field — single source of truth so the gold outline matches the
            # robot that actually emits the kick this tick.
            for rid, r in robots.items():
                c = (30, 144, 255) if int(rid) < TEAM_BLUE_SIZE else (255, 80, 80)
                if r.get("stale"):
                    c = C_STALE
                draw_robot(
                    screen, r["x"], r["y"], r["angle"], c,
                    is_attacker=(rid in attacker_rids),
                )
                if r.get("tag") is not None:
                    # Which AprilTag drives this robot (vision mirror).
                    lbl = _load_mono_font(12).render(f"T{r['tag']}", True, C_LINE)
                    sx, sy = w2s(r["x"], r["y"])
                    sy -= int(ROBOT_RADIUS * DISPLAY_SCALE) + 16
                    screen.blit(lbl, (sx - lbl.get_width() // 2, sy))

            if b:
                bx, by = w2s(b["x"], b["y"])
                bc = C_STALE if b.get("stale") and b.get("source") == "vision" else (230, 120, 0)
                _ = pygame.draw.circle(screen, bc, (bx, by), 5)

        if world_state and world_state.get("vision"):
            draw_vision_stats(screen, world_state["vision"])

        draw_confetti(screen, confetti_particles)
        # phase change popup
        phase_remaining = phase_popup_until - time.monotonic()
        if phase_remaining > 0.0:
            draw_phase_popup(screen, phase_popup_text, phase_remaining)

        remaining_flash = goal_flash_until - time.monotonic()
        if remaining_flash > 0.0:
            draw_goal_flash(
                screen, 
                goal_flash_team,
                goal_flash_score_text,
                remaining_flash,
            )

        draw_hud(
            screen, mode_idx, strategy_enabled, score_blue, score_red, game_info,
            rl_kick_on=rl_kick_enabled,
        )
        stuck_remaining = ball_stuck_popup_until - time.monotonic()
        if stuck_remaining > 0.0:
            draw_phase_popup(screen, "GAMESTUCK", stuck_remaining)
        if game_winner is not None:
            SND_WINNER.play()
            draw_winner_screen(screen, game_winner, score_blue, score_red)
        if show_controls:
            draw_controls_screen(screen, controls_close_rect)
        pygame.display.flip()
        _ = clock.tick(FPS)

    pygame.quit()


if __name__ == "__main__":
    main()
