"""Dribble skill — simulates dribbler bar behaviour.

Modeled as:
  1. Check the ball is inside the dribble capture zone (front of robot,
     slightly wider and shallower than the kick zone).
  2. Pull the ball toward the ideal dribble point (centre of front face).
  3. Damp the ball velocity to match the robot velocity — this simulates
     the backspin friction that stops the ball escaping sideways.
"""

import math
import os
import sys

import pymunk

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_PYTHON_DIR = os.path.join(_ROOT, "python")
if _PYTHON_DIR not in sys.path:
    sys.path.insert(0, _PYTHON_DIR)

from config import ROBOT_RADIUS 

DRIBBLE_ZONE_DEPTH      = 0.08   # metres — how far in front the zone extends
DRIBBLE_ZONE_HALF_WIDTH = 0.12   # metres — half-width of capture zone

# Spring-like pull toward the ideal contact point on the robot front face.
DRIBBLE_SPRING_K   = 18.0   # N/m  — stiffness of the capture spring
# Damping applied to ball velocity relative to robot — simulates backspin grip.
DRIBBLE_DAMPING    = 0.85   # fraction of relative velocity removed per tick


def _to_local(robot: pymunk.Body, wx: float, wy: float):
    """World-frame offset (wx, wy) → robot local frame (+x = forward)."""
    a = robot.angle
    lx =  math.cos(a) * wx + math.sin(a) * wy
    ly = -math.sin(a) * wx + math.cos(a) * wy
    return lx, ly


def _to_world(robot: pymunk.Body, lx: float, ly: float):
    """Robot local frame → world-frame vector."""
    a = robot.angle
    wx = math.cos(a) * lx - math.sin(a) * ly
    wy = math.sin(a) * lx + math.cos(a) * ly
    return wx, wy


def ball_in_dribble_zone(robot: pymunk.Body, ball: pymunk.Body) -> bool:
    """Return True if the ball is inside the dribbler capture zone."""
    dx = ball.position.x - robot.position.x
    dy = ball.position.y - robot.position.y
    lx, ly = _to_local(robot, dx, dy)

    zone_min_x = ROBOT_RADIUS
    zone_max_x = ROBOT_RADIUS + DRIBBLE_ZONE_DEPTH
    return zone_min_x <= lx <= zone_max_x and abs(ly) <= DRIBBLE_ZONE_HALF_WIDTH


def try_dribble_ball(robot: pymunk.Body, ball: pymunk.Body, dt: float) -> bool:
    """
    Apply dribbler forces for one physics tick.

    Returns True if the ball was in the dribble zone (dribbling active),
    False otherwise.
    """
    dx = ball.position.x - robot.position.x
    dy = ball.position.y - robot.position.y
    lx, ly = _to_local(robot, dx, dy)

    zone_min_x = ROBOT_RADIUS
    zone_max_x = ROBOT_RADIUS + DRIBBLE_ZONE_DEPTH

    # Widen capture slightly so ball can be scooped from just outside
    capture_min_x = ROBOT_RADIUS - 0.01
    capture_max_x = zone_max_x + 0.02
    if not (capture_min_x <= lx <= capture_max_x and abs(ly) <= DRIBBLE_ZONE_HALF_WIDTH + 0.02):
        return False

    # ── 1. Pull ball toward ideal contact point ──────────────────────────────
    # Ideal point: centre of robot front face in local frame.
    ideal_lx = ROBOT_RADIUS + DRIBBLE_ZONE_DEPTH * 0.3
    ideal_ly = 0.0

    error_lx = ideal_lx - lx
    error_ly = ideal_ly - ly

    # Convert spring force back to world frame and apply to ball.
    force_wx, force_wy = _to_world(robot, error_lx * DRIBBLE_SPRING_K,
                                          error_ly * DRIBBLE_SPRING_K)
    ball.apply_force_at_world_point((force_wx, force_wy), ball.position)

    # ── 2. Damp ball velocity relative to robot ──────────────────────────────
    rel_vx = ball.velocity.x - robot.velocity.x
    rel_vy = ball.velocity.y - robot.velocity.y

    # Convert relative velocity to local frame to selectively damp.
    rel_lx, rel_ly = _to_local(robot, rel_vx, rel_vy)

    # Damp forward (escape) component strongly, lateral component less.
    damped_lx = rel_lx * (1.0 - DRIBBLE_DAMPING)
    damped_ly = rel_ly * (1.0 - DRIBBLE_DAMPING * 0.5)

    new_rel_wx, new_rel_wy = _to_world(robot, damped_lx, damped_ly)
    ball.velocity = (
        robot.velocity.x + new_rel_wx,
        robot.velocity.y + new_rel_wy,
    )

    return True