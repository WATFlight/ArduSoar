"""State fusion (proposal 5).

Turns a raw ``SensorSnapshot`` (accel/gyro/GPS/compass/pitot/baro) into one
clean ``VehicleState`` that the guidance + thermal estimator consume. The
interface is fixed; today a trivial pass-through fills it, later a real EKF/AHRS
drops in behind the same ``update()`` signature.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass

from config import G
from sensors.interfaces import SensorSnapshot


@dataclass
class VehicleState:
    """Fused estimate the rest of the stack uses (mirrors GliderState plus
    ground velocity and vario, so guidance can run off real sensors)."""
    t: float
    x: float
    y: float
    h: float
    heading: float       # rad
    airspeed: float      # m/s
    bank: float          # rad
    vario: float         # vertical speed, m/s
    vx: float            # ground velocity
    vy: float


class StateFusion(ABC):
    @abstractmethod
    def update(self, snap: SensorSnapshot) -> VehicleState:
        ...


class PassthroughFusion(StateFusion):
    """Simplest possible fusion: trust each sensor directly. Good enough in sim;
    replace with an EKF/AHRS on hardware (noise, lag, GPS dropouts)."""

    def __init__(self):
        self._last = None  # retain last good values through GPS dropouts

    def update(self, snap: SensorSnapshot) -> VehicleState:
        heading = snap.compass.heading if snap.compass else (self._last.heading if self._last else 0.0)
        airspeed = snap.pitot.airspeed_longitudinal if snap.pitot else (self._last.airspeed if self._last else 0.0)
        vario = snap.baro.vertical_speed if snap.baro else 0.0
        # bank from lateral specific force: ay = g*tan(phi)
        bank = math.atan((snap.accel.ay / G)) if snap.accel else 0.0

        if snap.gps is not None:
            x, y, h = snap.gps.x, snap.gps.y, snap.gps.h
            vx, vy = snap.gps.vx, snap.gps.vy
        elif self._last is not None:
            # GPS dropout: dead-reckon roughly from airspeed + heading
            x, y, h = self._last.x, self._last.y, snap.baro.altitude if snap.baro else self._last.h
            vx = airspeed * math.cos(heading)
            vy = airspeed * math.sin(heading)
        else:
            x = y = h = vx = vy = 0.0

        st = VehicleState(snap.t, x, y, h, heading, airspeed, bank, vario, vx, vy)
        self._last = st
        return st
