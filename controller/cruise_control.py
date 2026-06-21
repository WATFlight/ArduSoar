"""Cruise control (Step 7 of the proposal): fly toward a fixed waypoint.

Extended with a periodic figure-8 search. The cruise schedule repeats:

    straight glide (straight_time)  ->  one figure-8  ->  straight glide  -> ...

Each figure-8 is exactly one right loop followed by one left loop, so the
glider sweeps out to *both* sides of the cruise line, then returns to course and
glides straight again before the next sweep. This lets it stumble onto thermals
that sit off the straight path without spiralling in place forever.

The figure-8 is an open-loop, time-based bank schedule: a constant-magnitude
bank whose sign flips after one full loop. One loop has radius
``R = V^2/(g*tan φ)`` and period ``T = 2πR/V``, so its lobes reach ~``2R`` out to
each side, and one figure-8 lasts two loop periods.
"""

import math

from glider_model.glider import GliderState
from controller.l1_guidance import L1Guidance
from config import G, DT


class CruiseControl:
    def __init__(
        self,
        waypoint_x: float,
        waypoint_y: float,
        l1: L1Guidance,
        straight_time: float = 18.0,
        search_bank_deg: float = 30.0,
        dt: float = DT,
    ):
        self.waypoint_x = waypoint_x
        self.waypoint_y = waypoint_y
        self.l1 = l1
        self.straight_time = straight_time
        self.search_bank = math.radians(search_bank_deg)
        self.dt = dt
        self._t = 0.0  # elapsed time spent in cruise mode

    def reset(self) -> None:
        """Restart the cruise schedule (straight glide, then figure-8 search).

        Called when the glider re-enters cruise after leaving a thermal, so it
        glides on course again before resuming the search pattern.
        """
        self._t = 0.0

    def command(self, glider_state: GliderState) -> float:
        self._t += self.dt

        phi = self.search_bank
        omega = G * math.tan(phi) / glider_state.V   # loop turn rate
        t_loop = 2.0 * math.pi / omega               # time for one full loop
        t_fig8 = 2.0 * t_loop                        # one figure-8 = right + left loop
        cycle = self.straight_time + t_fig8          # straight leg + one figure-8

        phase = self._t % cycle
        # Phase 1: glide straight toward the waypoint.
        if phase < self.straight_time:
            return self.l1.bank_to_point(glider_state, self.waypoint_x, self.waypoint_y)

        # Phase 2: exactly one figure-8 — right loop, then left loop — after
        # which `phase` wraps back into the straight leg.
        te = phase - self.straight_time
        return -phi if te < t_loop else phi
