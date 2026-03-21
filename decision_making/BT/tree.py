import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from geometry import distance
from state import GameState, RobotState


def _assign_roles(gamestate: GameState) -> dict[str, RobotState]:
    robots = sorted(gamestate.our_team, key=lambda r: distance(r.pos, gamestate.ball.pos))
    return {
        "attacker": robots[0],
        "supporter": robots[1],
        "defender": robots[2],
    }


def tick(gamestate: GameState, color: str) -> dict[int, dict]:
    from BT.attacker_tree import attacker_decide
    from BT.defender_tree import defender_decide
    from BT.supporter_tree import supporter_decide

    roles = _assign_roles(gamestate)
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