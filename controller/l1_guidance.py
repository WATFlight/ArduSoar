"""L1 nonlinear guidance (Step 6 of the proposal).

Given a target point, L1 computes a lateral acceleration that steers the
glider's velocity vector toward the target, then converts that acceleration to
a bank-angle command.

    a_cmd  = 2 * V^2 / L1 * sin(eta)
    phi_cmd = atan(a_cmd / g)

``eta`` is the *signed* angle from the velocity vector to the line-of-sight to
the target, positive counter-clockwise (to the left). With our heading
convention a positive bank turns left, so a target on the left (eta > 0)
correctly yields a positive (left) bank command.
"""

import math

from glider_model.glider import GliderState
from config import G, L1_DISTANCE, MAX_BANK_DEG


class L1Guidance:
    def __init__(self, L1: float = L1_DISTANCE, max_bank_deg: float = MAX_BANK_DEG):
        self.L1 = L1
        self.max_bank = math.radians(max_bank_deg)

    def bank_to_point(self, glider_state: GliderState, target_x: float, target_y: float) -> float:
        s = glider_state

        # Velocity direction and line-of-sight to the target.
        vx, vy = math.cos(s.heading), math.sin(s.heading)
        los_x, los_y = target_x - s.x, target_y - s.y

        # Signed angle from velocity to LOS via atan2(cross, dot).
        cross = vx * los_y - vy * los_x
        dot = vx * los_x + vy * los_y
        eta = math.atan2(cross, dot)

        a_cmd = 2.0 * s.V ** 2 / self.L1 * math.sin(eta)
        phi_cmd = math.atan(a_cmd / G)

        # Clamp to the safe bank range.
        return max(-self.max_bank, min(self.max_bank, phi_cmd))
