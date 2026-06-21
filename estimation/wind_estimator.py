"""Wind estimation (proposal 4).

Wind = ground velocity (GPS) − air velocity (airspeed along heading). Used to
(a) correct mapped thermal positions for downwind drift and (b) inform thermal
selection. Interface fixed; swap the simple difference for a filtered/least-
squares wind estimator later.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass

from estimation.state_fusion import VehicleState


@dataclass
class Wind:
    wx: float          # m/s, +x component
    wy: float
    speed: float
    direction: float   # rad, direction wind blows TOWARD


class WindEstimator(ABC):
    @abstractmethod
    def update(self, state: VehicleState) -> Wind:
        ...


class SimpleWindEstimator(WindEstimator):
    """Low-pass of (ground velocity − air velocity). ``alpha`` is the smoothing
    factor (0 = frozen, 1 = no smoothing)."""

    def __init__(self, alpha: float = 0.02):
        self.alpha = alpha
        self._wx = 0.0
        self._wy = 0.0
        self._init = False

    def update(self, state: VehicleState) -> Wind:
        air_x = state.airspeed * math.cos(state.heading)
        air_y = state.airspeed * math.sin(state.heading)
        wx = state.vx - air_x
        wy = state.vy - air_y
        if not self._init:
            self._wx, self._wy, self._init = wx, wy, True
        else:
            self._wx += self.alpha * (wx - self._wx)
            self._wy += self.alpha * (wy - self._wy)
        return Wind(self._wx, self._wy, math.hypot(self._wx, self._wy), math.atan2(self._wy, self._wx))
