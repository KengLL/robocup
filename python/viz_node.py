#!/usr/bin/env python3
"""
Visualization Node — single-robot manual control.

Subscribes to world state on VISION_PORT (ZMQ SUB).
Publishes targets          on MANUAL_PORT (ZMQ PUB).

Controls
--------
  Mouse click — send target for robot 0
  1 2 3       — controller: 1=PD inversion  2=MPC  3=Time-optimal (bang-bang)
"""
import os, sys, math, json
import pygame
import zmq

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from config import (
    VISION_PORT, MANUAL_PORT,
    FIELD_W, FIELD_H, DISPLAY_W, DISPLAY_H, DISPLAY_SCALE,
    ROBOT_RADIUS, WHEEL_ANGLES, FPS, ARRIVAL_THRESH,
)

# ── Palette ───────────────────────────────────────────────────────────────────
C_FIELD   = (0,   100,   0)
C_LINE    = (255, 255, 255)
C_ROBOT   = ( 30, 144, 255)
C_WHEEL   = (200,   0,   0)
C_HUD_BG  = ( 30,  30,  30)

MODES = ["2005_INVERSION", "2005_TIME_OPTIMAL", "MPC"]
MODE_COLORS = {
    "2005_INVERSION":    ( 0, 255, 128),
    "MPC":               ( 0, 200, 255),
    "2005_TIME_OPTIMAL": (220,  80, 255),
}
MODE_KEYS = {pygame.K_1: 0, pygame.K_2: 1, pygame.K_3: 2}

HUD_MODE_LABELS = {
    "2005_INVERSION":    "PD",
    "2005_TIME_OPTIMAL": "TIME",
    "MPC":               "MPC",
}

HUD_H = 26

PIXEL_GLYPHS = {
    "P": ["111", "101", "111", "100", "100"],
    "D": ["110", "101", "101", "101", "110"],
    "T": ["111", "010", "010", "010", "010"],
    "I": ["111", "010", "010", "010", "111"],
    "M": ["101", "111", "111", "101", "101"],
    "E": ["111", "100", "111", "100", "111"],
    "C": ["111", "100", "100", "100", "111"],
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
    pygame.draw.line(surf, C_LINE,
                     (DISPLAY_W // 2, 0), (DISPLAY_W // 2, DISPLAY_H), 2)
    pygame.draw.circle(surf, C_LINE,
                       (DISPLAY_W // 2, DISPLAY_H // 2),
                       int(0.5 * DISPLAY_SCALE), 2)


def draw_robot(surf: pygame.Surface,
               x: float, y: float, angle: float) -> None:
    cx, cy = w2s(x, y)
    r = max(4, int(ROBOT_RADIUS * DISPLAY_SCALE))
    pygame.draw.circle(surf, C_ROBOT, (cx, cy), r)
    pygame.draw.circle(surf, C_LINE,  (cx, cy), r, 2)
    # Heading arrow
    ex = int(cx + math.cos(angle) * r * 1.4)
    ey = int(cy - math.sin(angle) * r * 1.4)
    pygame.draw.line(surf, C_LINE, (cx, cy), (ex, ey), 3)
    # Wheel dots
    for alpha in WHEEL_ANGLES:
        wx = int(cx + math.cos(angle + alpha) * r)
        wy = int(cy - math.sin(angle + alpha) * r)
        pygame.draw.circle(surf, C_WHEEL, (wx, wy), 4)


def draw_dotted_line(surf: pygame.Surface,
                     x0: float, y0: float,
                     x1: float, y1: float,
                     color: tuple, spacing: int = 12) -> None:
    sx0, sy0 = w2s(x0, y0)
    sx1, sy1 = w2s(x1, y1)
    dx, dy   = sx1 - sx0, sy1 - sy0
    length   = math.hypot(dx, dy)
    if length < 1:
        return
    steps = max(1, int(length / spacing))
    for i in range(steps + 1):
        t  = i / steps
        px = int(sx0 + dx * t)
        py = int(sy0 + dy * t)
        pygame.draw.circle(surf, color, (px, py), 2)


def draw_pin(surf: pygame.Surface,
             x: float, y: float, color: tuple) -> None:
    """Map-pin icon: filled circle head + vertical stem."""
    cx, cy    = w2s(x, y)
    stem_top  = cy - 22
    head_r    = 7
    # Stem
    pygame.draw.line(surf, color, (cx, cy), (cx, stem_top + head_r), 2)
    # Head
    pygame.draw.circle(surf, color,         (cx, stem_top), head_r)
    pygame.draw.circle(surf, (255, 255, 255),(cx, stem_top), head_r, 1)


def draw_pixel_text(surf: pygame.Surface, text: str, x: int, y: int,
                    color: tuple[int, int, int], pixel: int = 2, spacing: int = 1) -> None:
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


def draw_hud(surf: pygame.Surface, mode_idx: int) -> None:
    """Bottom status bar — mode selector blocks only."""
    y0  = DISPLAY_H
    pad = 5
    pygame.draw.rect(surf, C_HUD_BG, (0, y0, DISPLAY_W, HUD_H))

    block_w = 40
    x = pad
    active_mode = MODES[mode_idx]

    for mode in MODES:
        active  = (mode == active_mode)
        color   = MODE_COLORS[mode] if active else tuple(v // 4 for v in MODE_COLORS[mode])
        pygame.draw.rect(surf, color,
                         (x, y0 + pad, block_w, HUD_H - pad * 2), border_radius=3)
        if active:
            pygame.draw.rect(surf, (255, 255, 255),
                             (x, y0 + pad, block_w, HUD_H - pad * 2), 1, border_radius=3)

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


# ── main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    pygame.init()
    screen = pygame.display.set_mode((DISPLAY_W, DISPLAY_H + HUD_H))
    pygame.display.set_caption("RoboCup — click to move  |  1/2/3: PD / TIME / MPC")
    clock  = pygame.time.Clock()

    ctx = zmq.Context()

    vision_sub = ctx.socket(zmq.SUB)
    vision_sub.connect(f"tcp://localhost:{VISION_PORT}")
    vision_sub.setsockopt_string(zmq.SUBSCRIBE, "")
    vision_sub.setsockopt(zmq.RCVTIMEO, 0)

    manual_pub = ctx.socket(zmq.PUB)
    manual_pub.bind(f"tcp://*:{MANUAL_PORT}")

    world_state: dict | None = None
    mode_idx = 0

    # Overlay state: cleared on arrival
    target_pin:  tuple[float, float] | None = None
    path_start:  tuple[float, float] | None = None

    print("[VizNode] Click field to move robot  |  1=PD  2=TIME  3=MPC")

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                if event.key in MODE_KEYS:
                    mode_idx = MODE_KEYS[event.key]
                    print(f"[VizNode] Mode → {MODES[mode_idx]}")

            elif event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = event.pos
                if my < DISPLAY_H:          # ignore clicks on HUD
                    wx, wy = s2w(mx, my)
                    mode   = MODES[mode_idx]
                    # Snapshot robot position as path start
                    if world_state and "0" in world_state.get("robots", {}):
                        r = world_state["robots"]["0"]
                        path_start = (r["x"], r["y"])
                    else:
                        path_start = None
                    target_pin = (wx, wy)
                    manual_pub.send_string(json.dumps({
                        "targets": {"0": {"x": wx, "y": wy, "mode": mode}}
                    }))
                    print(f"[VizNode] Target → ({wx:.2f}, {wy:.2f})  [{mode}]")

        # Drain vision (keep latest frame)
        while True:
            try:
                world_state = json.loads(vision_sub.recv_string())
            except zmq.Again:
                break

        # Auto-clear overlay when robot arrives
        if target_pin and world_state:
            r  = world_state["robots"].get("0")
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
                draw_dotted_line(screen,
                                 path_start[0], path_start[1],
                                 target_pin[0],  target_pin[1],
                                 mode_color)
            draw_pin(screen, target_pin[0], target_pin[1], mode_color)

        if world_state:
            r = world_state["robots"].get("0")
            if r:
                draw_robot(screen, r["x"], r["y"], r["angle"])

        draw_hud(screen, mode_idx)

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()


if __name__ == "__main__":
    main()
