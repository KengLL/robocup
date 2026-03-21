import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from geometry import distance, our_goal
from state import GameState, RobotState

#assign roles based on distance to ball and goal
def _assign_roles(gamestate: GameState, color: str) -> dict[str, RobotState]:
    goal = our_goal(color)
    robots = gamestate.our_team

    # attacker = closest to ball
    attacker = min(robots, key=lambda r: distance(r.pos, gamestate.ball.pos))

    # defender = closest to own goal (excluding attacker)
    remaining = [r for r in robots if r.id != attacker.id]
    defender = min(remaining, key=lambda r: distance(r.pos, goal))

    # supporter = whoever is left
    supporter = next(r for r in remaining if r.id != defender.id)

    return {
        "attacker": attacker,
        "supporter": supporter,
        "defender": defender,
    }


def tick(gamestate: GameState, color: str) -> dict[int, dict]:
    from BT.attacker_tree import attacker_decide
    from BT.defender_tree import defender_decide
    from BT.supporter_tree import supporter_decide

    roles = _assign_roles(gamestate, color)
    targets = {}

    attacker = roles["attacker"]
    pos, kick = attacker_decide(attacker, gamestate, color)
    targets[attacker.id] = {"pos": pos, "kick": kick}

    supporter = roles["supporter"]
    pos, kick = supporter_decide(supporter, gamestate, color)
    targets[supporter.id] = {"pos": pos, "kick": kick}

    defender = roles["defender"]
    pos, kick = defender_decide(defender, gamestate, color)
    targets[defender.id] = {"pos": pos, "kick": kick}

    return targets