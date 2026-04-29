from __future__ import annotations
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from geometry import can_pass_ball, point_to_line_distance
from state import RobotState

GOAL_WIDTH = 1.0
MIN_ROBOT_SEPARATION = 2.0

def best_shot_target(robot: RobotState, opponents: list, goal: np.ndarray) -> np.ndarray | None:
    if not opponents:
        return goal.copy()

    # check 3 points — center, left, right
    targets = [
        goal,
        goal + np.array([0.0,  GOAL_WIDTH * 0.4]),
        goal + np.array([0.0, -GOAL_WIDTH * 0.4]),
    ]

    best_target = None
    best_clearance = 0.15

    for target in targets:
        min_dist = min(
            point_to_line_distance(opp.pos, robot.pos, target)
            for opp in opponents
        )
        if min_dist > best_clearance:
            best_clearance = min_dist
            best_target = target

    return best_target


def teammate_open(robot: RobotState, teammate: RobotState, opponents: list) -> bool:
    return can_pass_ball(robot, teammate, opponents, block_radius=0.3)

def avoid_teammates(target_pos, robot, gamestate):
    adjusted = target_pos.copy()
    for teammate in gamestate.our_team:
        if teammate.id == robot.id:
            continue
        diff = adjusted - teammate.pos
        dist = np.linalg.norm(diff)
        if dist < MIN_ROBOT_SEPARATION and dist > 1e-6:
            adjusted += (diff / dist) * (MIN_ROBOT_SEPARATION - dist)
    return adjusted