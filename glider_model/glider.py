"""Simple kinematic glider model (Step 2 of the proposal).

This is a point-mass / coordinated-turn kinematic model. We do not model real
aircraft dynamics, actuators, or wind. Airspeed is held constant; the only
control input is the commanded bank angle.

Heading convention: measured counter-clockwise from the +x axis, so the
velocity vector is ``(V*cos(heading), V*sin(heading))``. A positive bank angle
produces a positive heading rate, i.e. a left (counter-clockwise) turn.
"""

import math
from dataclasses import dataclass

from config import G, BASE_SINK_RATE


@dataclass
class GliderState:
    """Mutable glider state."""

    x: float          # east position (m)
    y: float          # north position (m)
    h: float          # altitude (m)
    heading: float    # direction of travel (rad, CCW from +x)
    V: float          # airspeed == ground speed in the no-wind version (m/s)
    bank_angle: float = 0.0  # current roll/bank angle phi (rad)


class SimpleGlider:
    def __init__(self, state: GliderState, base_sink_rate: float = BASE_SINK_RATE):
        self.state = state
        self.base_sink_rate = base_sink_rate

    def sink_rate(self) -> float:
        """Sink rate grows with bank because the wing must carry more load.

            load_factor = 1 / cos(phi)
            sink_rate   = base_sink_rate * load_factor
        """
        phi = self.state.bank_angle
        load_factor = 1.0 / max(math.cos(phi), 1e-3)
        return self.base_sink_rate * load_factor

    def step(self, bank_command: float, thermal_lift: float, dt: float) -> float:
        """Advance the state by ``dt`` and return the net climb rate ``h_dot``.

        ``bank_command`` is the commanded bank angle (rad); for this kinematic
        model we apply it instantly (no actuator lag). ``thermal_lift`` is the
        true vertical air velocity ``w`` at the glider's current location.
        """
        s = self.state
        s.bank_angle = bank_command

        # Coordinated-turn heading rate: heading_dot = g * tan(phi) / V
        heading_dot = G * math.tan(s.bank_angle) / s.V
        s.heading += heading_dot * dt

        # Horizontal kinematics
        s.x += s.V * math.cos(s.heading) * dt
        s.y += s.V * math.sin(s.heading) * dt

        # Vertical: net climb = thermal lift - sink rate
        h_dot = thermal_lift - self.sink_rate()
        s.h += h_dot * dt

        return h_dot
