"""Kick skill — physics-level kick zone check and impulse application."""

import math
import os
import sys

import pymunk

# Resolve shared constants from python/config.py regardless of caller cwd.
_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_PYTHON_DIR = os.path.join(_ROOT, "python")
if _PYTHON_DIR not in sys.path:
    sys.path.insert(0, _PYTHON_DIR)

from config import ROBOT_RADIUS

# Kick model: small rectangular contact zone in front of robot.
KICK_ZONE_DEPTH = 0.12
KICK_ZONE_HALF_WIDTH = 0.10
KICK_IMPULSE = 0.22


def try_kick_ball(robot: pymunk.Body, ball: pymunk.Body) -> bool:
    """Kick if ball is in a small front tangent zone of the robot."""
    dx = ball.position.x - robot.position.x
    dy = ball.position.y - robot.position.y
    a = robot.angle

    # World -> robot local frame where +x is robot forward.
    local_x = math.cos(a) * dx + math.sin(a) * dy
    local_y = -math.sin(a) * dx + math.cos(a) * dy

    zone_min_x = ROBOT_RADIUS
    zone_max_x = ROBOT_RADIUS + KICK_ZONE_DEPTH
    in_zone = zone_min_x <= local_x <= zone_max_x and abs(local_y) <= KICK_ZONE_HALF_WIDTH
    if not in_zone:
        return False

    impulse = (math.cos(a) * KICK_IMPULSE, math.sin(a) * KICK_IMPULSE)
    ball.apply_impulse_at_world_point(impulse, ball.position)
    return True
