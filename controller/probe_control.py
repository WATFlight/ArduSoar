"""Probe control (Step 8 of the proposal).

A simple bang-bang controller used while the thermal estimate exists but is not
yet trusted enough to commit to circling. It steers toward the estimated core
with a fixed bank, producing exploratory, curving behaviour.
"""

import math

from glider_model.glider import GliderState
from thermal_estimator.estimator import ThermalEstimate


class ProbeControl:
    def __init__(self, fixed_bank_deg: float = 25.0):
        self.fixed_bank = math.radians(fixed_bank_deg)

    def command(self, glider_state: GliderState, estimate: ThermalEstimate) -> float:
        s = glider_state

        # Is the estimated core to the left or right of the velocity vector?
        vx, vy = math.cos(s.heading), math.sin(s.heading)
        los_x, los_y = estimate.x_c - s.x, estimate.y_c - s.y
        cross = vx * los_y - vy * los_x  # > 0 -> core is to the left

        # Positive bank turns left, so steer toward the core.
        return self.fixed_bank if cross > 0 else -self.fixed_bank
