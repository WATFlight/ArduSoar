"""Circling control (Step 9 of the proposal).

In THERMAL mode the glider circles the estimated core. The basic version picks
a radius of ``0.5 * R_th`` (clamped), places a lookahead point a little further
around the circle, and tracks it with L1 guidance.

An optional grid-search optimiser can instead pick the radius that maximises
net climb ``w(R) - sink_rate(phi)``.
"""

import math

from glider_model.glider import GliderState
from thermal_estimator.estimator import ThermalEstimate
from controller.l1_guidance import L1Guidance
from config import G, BASE_SINK_RATE

R_MIN = 10.0  # smallest allowed circling radius (m)


class CirclingControl:
    def __init__(self, l1: L1Guidance, optimize: bool = False, base_sink_rate: float = BASE_SINK_RATE):
        self.l1 = l1
        self.optimize = optimize
        self.base_sink_rate = base_sink_rate

    def choose_radius(self, estimate: ThermalEstimate, V: float) -> float:
        """Desired circle radius, either the simple rule or the optimiser."""
        if self.optimize:
            return self._optimal_radius(estimate, V)
        r = 0.5 * estimate.R_th
        return self._clamp_radius(r, estimate)

    def command(self, glider_state: GliderState, estimate: ThermalEstimate) -> float:
        s = glider_state
        R = self.choose_radius(estimate, s.V)

        # Angle of the glider as seen from the estimated core.
        theta = math.atan2(s.y - estimate.y_c, s.x - estimate.x_c)

        # Advance the angle (counter-clockwise) to form a lookahead point on the
        # circle. The arc length is tied to the L1 distance so the lookahead
        # scales sensibly with the chosen radius.
        delta = max(0.3, min(self.l1.L1 / R, 1.2))
        theta_target = theta + delta

        target_x = estimate.x_c + R * math.cos(theta_target)
        target_y = estimate.y_c + R * math.sin(theta_target)
        return self.l1.bank_to_point(s, target_x, target_y)

    # -- helpers ------------------------------------------------------------
    def _clamp_radius(self, r: float, estimate: ThermalEstimate) -> float:
        r_max = 2.0 * estimate.R_th
        return max(R_MIN, min(r, r_max))

    def _optimal_radius(self, estimate: ThermalEstimate, V: float) -> float:
        """Grid search: pick the radius giving the best net climb at speed V."""
        r_max = 2.0 * estimate.R_th
        best_r, best_climb = R_MIN, -1e9
        steps = 40
        for i in range(steps + 1):
            R = R_MIN + (r_max - R_MIN) * i / steps
            w = estimate.W_0 * math.exp(-(R ** 2) / (estimate.R_th ** 2))
            phi = math.atan(V ** 2 / (G * R))           # bank needed for radius R
            sink = self.base_sink_rate / max(math.cos(phi), 1e-3)
            net = w - sink
            if net > best_climb:
                best_climb, best_r = net, R
        return best_r
