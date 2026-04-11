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

import json
import math
import os
import random
import sys
import time

import pygame
import zmq

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from config import (
    ARRIVAL_THRESH,
    DISPLAY_H,
    DISPLAY_SCALE,
    DISPLAY_W,
    FIELD_H,
    FIELD_W,
    FPS,
    MANUAL_PORT,
    PX_PER_METER,
    ROBOT_RADIUS,
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

MODES = ["2005_INVERSION", "2005_TIME_OPTIMAL", "MPC", "MANUAL"]
MODE_COLORS = {
    "2005_INVERSION": (0, 255, 128),
    "MPC": (0, 200, 255),
    "2005_TIME_OPTIMAL": (220, 80, 255),
    "MANUAL": (255, 165, 0),
}
MODE_KEYS = {pygame.K_1: 0, pygame.K_2: 1, pygame.K_3: 2, pygame.K_4: 3}

HUD_MODE_LABELS = {
    "2005_INVERSION": "PD",
    "2005_TIME_OPTIMAL": "TIME",
    "MPC": "MPC",
    "MANUAL": "CTL",
}

# Strategy toggle
C_STRAT_ON = (0, 255, 0)
C_STRAT_OFF = (255, 60, 60)

# Manual / gamepad settings
MANUAL_MAX_OMEGA = 5.0   # rad/s for full stick deflection
JOYSTICK_DEADZONE = 0.10
# Xbox controller axis indices (adjust if your OS maps them differently):
#   Axis 0 = Left stick X,  Axis 1 = Left stick Y (up = -1)
#   Axis 2 = Right stick X  (right = +1 → clockwise → negative ω)
JS_AXIS_LX, JS_AXIS_LY, JS_AXIS_RX = 0, 1, 2

HUD_H = 26

# Goal mouth matches simulation/godot geometry: 200 px at 140 px/m.
GOAL_MOUTH_H = 200.0 / PX_PER_METER
GOAL_Y_MIN = (FIELD_H - GOAL_MOUTH_H) / 2.0
GOAL_Y_MAX = GOAL_Y_MIN + GOAL_MOUTH_H

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
}


# ── helpers ───────────────────────────────────────────────────────────────────


def w2s(x: float, y: float) -> tuple[int, int]:
    """World metres (y-up) → screen pixels (y-down)."""
    return int(x * DISPLAY_SCALE), int(DISPLAY_H - y * DISPLAY_SCALE)


def s2w(px: int, py: int) -> tuple[float, float]:
    """Screen pixels → world metres (clamped to field)."""
    wx = max(0.0, min(FIELD_W, px / DISPLAY_SCALE))
    wy = max(0.0, min(FIELD_H, (DISPLAY_H - py) / DISPLAY_SCALE))
    return wx, wy


# ── drawing ───────────────────────────────────────────────────────────────────


def draw_field(surf: pygame.Surface) -> None:
    surf.fill(C_FIELD)
    pygame.draw.rect(surf, C_LINE, (0, 0, DISPLAY_W, DISPLAY_H), 3)
    pygame.draw.line(surf, C_LINE, (DISPLAY_W // 2, 0), (DISPLAY_W // 2, DISPLAY_H), 2)
    pygame.draw.circle(
        surf, C_LINE, (DISPLAY_W // 2, DISPLAY_H // 2), int(0.5 * DISPLAY_SCALE), 2
    )

    # Goal-post markers overlaid at the side edges.
    _, gy0 = w2s(0.0, GOAL_Y_MAX)
    _, gy1 = w2s(0.0, GOAL_Y_MIN)
    # Extra white underlay on blue post for stronger contrast against grass.
    pygame.draw.line(surf, (255, 255, 255), (0, gy0), (0, gy1), 8)
    pygame.draw.line(surf, C_GOAL_LEFT, (0, gy0), (0, gy1), 5)
    pygame.draw.line(surf, C_GOAL_RIGHT, (DISPLAY_W - 1, gy0), (DISPLAY_W - 1, gy1), 6)


def draw_robot(
    surf: pygame.Surface, x: float, y: float, angle: float, color: tuple
) -> None:
    cx, cy = w2s(x, y)
    r = max(4, int(ROBOT_RADIUS * DISPLAY_SCALE))
    pygame.draw.circle(surf, color, (cx, cy), r)
    pygame.draw.circle(surf, C_LINE, (cx, cy), r, 2)
    # Heading arrow
    ex = int(cx + math.cos(angle) * r * 1.4)
    ey = int(cy - math.sin(angle) * r * 1.4)
    pygame.draw.line(surf, C_LINE, (cx, cy), (ex, ey), 3)
    # Wheel dots
    for alpha in WHEEL_ANGLES:
        wx = int(cx + math.cos(angle + alpha) * r)
        wy = int(cy - math.sin(angle + alpha) * r)
        pygame.draw.circle(surf, C_WHEEL, (wx, wy), 4)


def draw_dotted_line(
    surf: pygame.Surface,
    x0: float,
    y0: float,
    x1: float,
    y1: float,
    color: tuple,
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
        pygame.draw.circle(surf, color, (px, py), 2)


def draw_pin(surf: pygame.Surface, x: float, y: float, color: tuple) -> None:
    """Map-pin icon: filled circle head + vertical stem."""
    cx, cy = w2s(x, y)
    stem_top = cy - 22
    head_r = 7
    # Stem
    pygame.draw.line(surf, color, (cx, cy), (cx, stem_top + head_r), 2)
    # Head
    pygame.draw.circle(surf, color, (cx, stem_top), head_r)
    pygame.draw.circle(surf, (255, 255, 255), (cx, stem_top), head_r, 1)


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
                    pygame.draw.rect(
                        surf,
                        color,
                        (cursor_x + col * pixel, y + row * pixel, pixel, pixel),
                    )
        cursor_x += (3 * pixel) + spacing + pixel


def draw_hud(
    surf: pygame.Surface,
    mode_idx: int,
    strategy_on: bool = False,
    score_blue: int = 0,
    score_red: int = 0,
) -> None:
    """Bottom status bar — mode selector blocks + centered score + strategy toggle."""
    y0 = DISPLAY_H
    pad = 5
    pygame.draw.rect(surf, C_HUD_BG, (0, y0, DISPLAY_W, HUD_H))

    block_w = 40
    x = pad
    active_mode = MODES[mode_idx]

    for mode in MODES:
        active = mode == active_mode
        color = (
            MODE_COLORS[mode] if active else tuple(v // 4 for v in MODE_COLORS[mode])
        )
        pygame.draw.rect(
            surf, color, (x, y0 + pad, block_w, HUD_H - pad * 2), border_radius=3
        )
        if active:
            pygame.draw.rect(
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

    # Strategy toggle indicator (right side)
    strat_label = "AUTO ON" if strategy_on else "AUTO OFF"
    strat_color = C_STRAT_ON if strategy_on else C_STRAT_OFF
    pixel = 2
    glyph_w = 3 * pixel
    char_step = glyph_w + 1 + pixel
    total_w = len(strat_label) * char_step - pixel
    sx = DISPLAY_W - total_w - pad
    sy = y0 + (HUD_H - 5 * pixel) // 2
    draw_pixel_text(surf, strat_label, sx, sy, strat_color, pixel=pixel, spacing=1)


def _spawn_confetti(particles: list, team: str) -> None:
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


def _update_confetti(particles: list, dt: float) -> None:
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


def draw_confetti(surf: pygame.Surface, particles: list) -> None:
    for p in particles:
        if p["x"] < 0 or p["x"] >= DISPLAY_W or p["y"] < 0 or p["y"] >= DISPLAY_H:
            continue
        scale = max(0.35, p["life"] / p["max_life"])
        radius = max(1, int(p["size"] * scale))
        pygame.draw.circle(surf, p["color"], (int(p["x"]), int(p["y"])), radius)


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
    tint.fill((team_color[0], team_color[1], team_color[2], alpha))
    surf.blit(tint, (0, 0))

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


# ── main ──────────────────────────────────────────────────────────────────────

def _apply_deadzone(v: float, dz: float) -> float:
    return 0.0 if abs(v) < dz else v



def main() -> None:
    pygame.init()
    pygame.joystick.init()
    joystick = None
    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"[VizNode] Gamepad detected: {joystick.get_name()}")

    screen = pygame.display.set_mode((DISPLAY_W, DISPLAY_H + HUD_H))
    pygame.display.set_caption(
        "RoboCup — click to move  |  1/2/3: PD/TIME/MPC  |  4: MANUAL (WASD+QE / gamepad)  |  Enter: Kick"
    )
    clock = pygame.time.Clock()

    ctx = zmq.Context()

    vision_sub = ctx.socket(zmq.SUB)
    vision_sub.connect(f"tcp://localhost:{VISION_PORT}")
    vision_sub.setsockopt_string(zmq.SUBSCRIBE, "")
    vision_sub.setsockopt(zmq.RCVTIMEO, 0)

    manual_pub = ctx.socket(zmq.PUB)
    manual_pub.bind(f"tcp://*:{MANUAL_PORT}")

    world_state: dict | None = None
    score_blue = 0
    score_red = 0
    last_goal_seq_seen = -1
    goal_flash_team = "blue"
    goal_flash_until = 0.0
    goal_flash_score_text = "0-0"
    confetti_particles: list = []

    mode_idx = 0
    prev_mode_idx = 0
    strategy_enabled = False

    # Overlay state: cleared on arrival
    target_pin: tuple[float, float] | None = None
    path_start: tuple[float, float] | None = None

    print(
        "[VizNode] Click field to move robot  |  1=PD  2=TIME  3=MPC  "
        "4=MANUAL  5=Toggle Strategy  Enter=Kick"
    )

    running = True
    while running:
        dt_s = 1.0 / FPS
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_5:
                    strategy_enabled = not strategy_enabled
                    manual_pub.send_string(json.dumps(
                        {"strategy_enabled": strategy_enabled}
                    ))
                    state_str = "ON" if strategy_enabled else "OFF"
                    print(f"[VizNode] Strategy → {state_str}")
                elif event.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                    manual_pub.send_string(json.dumps({"kick": {"0": True}}))
                    print("[VizNode] Kick requested for robot 0")
                elif event.key in MODE_KEYS:
                    prev_mode_idx = mode_idx
                    mode_idx = MODE_KEYS[event.key]
                    print(f"[VizNode] Mode → {MODES[mode_idx]}")
                    # Leaving MANUAL: send zero velocity so robot stops immediately
                    if MODES[prev_mode_idx] == "MANUAL" and mode_idx != prev_mode_idx:
                        manual_pub.send_string(json.dumps(
                            {"direct": {"0": {"vx": 0.0, "vy": 0.0, "w": 0.0}}}
                        ))

            elif event.type == pygame.MOUSEBUTTONDOWN:
                if MODES[mode_idx] != "MANUAL":
                    mx, my = event.pos
                    if my < DISPLAY_H:  # ignore clicks on HUD
                        wx, wy = s2w(mx, my)
                        mode = MODES[mode_idx]
                        # Snapshot robot position as path start
                        if world_state and "0" in world_state.get("robots", {}):
                            r = world_state["robots"]["0"]
                            path_start = (r["x"], r["y"])
                        else:
                            path_start = None
                        target_pin = (wx, wy)
                        manual_pub.send_string(
                            json.dumps({"targets": {"0": {"x": wx, "y": wy, "mode": mode}}})
                        )
                        print(f"[VizNode] Target → ({wx:.2f}, {wy:.2f})  [{mode}]")

        # ── MANUAL mode: read keyboard / gamepad and publish direct velocity ───
        if MODES[mode_idx] == "MANUAL":
            target_pin = None
            path_start = None

            keys = pygame.key.get_pressed()
            vx = (1.0 if keys[pygame.K_d] else 0.0) - (1.0 if keys[pygame.K_a] else 0.0)
            vy = (1.0 if keys[pygame.K_w] else 0.0) - (1.0 if keys[pygame.K_s] else 0.0)
            w = (1.0 if keys[pygame.K_q] or keys[pygame.K_j] else 0.0) \
                - (1.0 if keys[pygame.K_e] or keys[pygame.K_k] else 0.0)
            w *= MANUAL_MAX_OMEGA

            if joystick is not None:
                lx = _apply_deadzone(joystick.get_axis(JS_AXIS_LX), JOYSTICK_DEADZONE)
                ly = _apply_deadzone(joystick.get_axis(JS_AXIS_LY), JOYSTICK_DEADZONE)
                rx = _apply_deadzone(joystick.get_axis(JS_AXIS_RX), JOYSTICK_DEADZONE)
                # Left stick overrides keyboard if any gamepad input detected
                if abs(lx) > 0 or abs(ly) > 0 or abs(rx) > 0:
                    vx = lx
                    vy = -ly                      # SDL Y axis is inverted (up = -1)
                    w = -rx * MANUAL_MAX_OMEGA    # right stick right = clockwise = -ω

            manual_pub.send_string(json.dumps(
                {"direct": {"0": {"vx": vx, "vy": vy, "w": w}}}
            ))

        # Drain vision (keep latest frame)
        while True:
            try:
                world_state = json.loads(vision_sub.recv_string())
            except zmq.Again:
                break

        if world_state:
            score = world_state.get("score", {})
            score_blue = int(score.get("blue", score_blue))
            score_red = int(score.get("red", score_red))

            last_goal = world_state.get("last_goal")
            if isinstance(last_goal, dict):
                seq = int(last_goal.get("seq", -1))
                if seq > last_goal_seq_seen:
                    last_goal_seq_seen = seq
                    scored_post_team = str(last_goal.get("team", "blue"))
                    # Reverse effect color relative to post scored on:
                    # score on red post -> blue flash, score on blue post -> red flash.
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
            for rid, r in world_state.get("robots", {}).items():
                c = (30, 144, 255) if int(rid) < 3 else (255, 80, 80)
                draw_robot(screen, r["x"], r["y"], r["angle"], c)

            b = world_state.get("ball")
            if b:
                bx, by = w2s(b["x"], b["y"])
                pygame.draw.circle(screen, (230, 120, 0), (bx, by), 5)

        draw_confetti(screen, confetti_particles)

        remaining_flash = goal_flash_until - time.monotonic()
        if remaining_flash > 0.0:
            draw_goal_flash(
                screen,
                goal_flash_team,
                goal_flash_score_text,
                remaining_flash,
            )

        draw_hud(screen, mode_idx, strategy_enabled, score_blue, score_red)

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()


if __name__ == "__main__":
    main()
