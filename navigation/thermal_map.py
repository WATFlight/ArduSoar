"""Thermal map + reachability + scoring (proposals 2 & 5).

A persistent map of discovered thermals (in GPS/map coordinates). Each new
estimate is merged into the nearest known thermal or added as a new one. The map
answers the questions cross-country planning needs:

  * which thermals can I still glide to from here?  (reachability)
  * which of those is worth going to?               (score)

This replaces "search blindly, forget after leaving" with "remember, plan".
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class MappedThermal:
    x: float
    y: float
    strength: float       # W_0 estimate
    radius: float         # R_th estimate
    last_seen: float      # time
    n_obs: int            # how many times observed (confidence proxy)


class ThermalMap:
    def __init__(self, merge_dist: float = 80.0, freshness_tau: float = 600.0):
        self.merge_dist = merge_dist      # estimates closer than this = same thermal
        self.freshness_tau = freshness_tau  # score decay time constant (s)
        self.thermals: list[MappedThermal] = []

    # -- building the map ---------------------------------------------------
    def add_or_update(self, x: float, y: float, strength: float, radius: float, t: float) -> MappedThermal:
        """Merge a new estimate into the nearest known thermal, or add it."""
        nearest, dmin = None, float("inf")
        for th in self.thermals:
            d = math.hypot(th.x - x, th.y - y)
            if d < dmin:
                nearest, dmin = th, d
        if nearest is not None and dmin <= self.merge_dist:
            # running average, weighted toward accumulated observations
            k = 1.0 / (nearest.n_obs + 1)
            nearest.x += k * (x - nearest.x)
            nearest.y += k * (y - nearest.y)
            nearest.strength += k * (strength - nearest.strength)
            nearest.radius += k * (radius - nearest.radius)
            nearest.n_obs += 1
            nearest.last_seen = t
            return nearest
        new = MappedThermal(x, y, strength, radius, t, 1)
        self.thermals.append(new)
        return new

    def drift_with_wind(self, wind, dt: float) -> None:
        """Proposal 4 hook: advect mapped thermals downwind over dt seconds."""
        for th in self.thermals:
            th.x += wind.wx * dt
            th.y += wind.wy * dt

    def mark_dead(self, x: float, y: float, radius: float = None) -> None:
        """We flew here and found no lift: drop the mapped thermal (it died)."""
        r = radius if radius is not None else self.merge_dist
        self.thermals = [th for th in self.thermals if math.hypot(th.x - x, th.y - y) > r]

    # -- querying for planning ---------------------------------------------
    def score(self, th: MappedThermal, now: float) -> float:
        """Higher = more worth going to: strong, fresh, repeatedly seen."""
        freshness = math.exp(-(now - th.last_seen) / self.freshness_tau)
        confidence = th.n_obs / (th.n_obs + 3.0)
        return th.strength * freshness * confidence

    def reachable(self, x: float, y: float, altitude: float, glide_ratio: float = 22.0,
                  reserve: float = 100.0) -> list[MappedThermal]:
        """Thermals within glide range from (x, y) at this altitude."""
        usable = max(0.0, altitude - reserve)
        max_glide = usable * glide_ratio
        return [th for th in self.thermals if math.hypot(th.x - x, th.y - y) <= max_glide]

    def best_reachable(self, x: float, y: float, altitude: float, now: float,
                       glide_ratio: float = 22.0, reserve: float = 100.0) -> MappedThermal | None:
        """Pick the highest-scoring thermal you can still glide to (a simple
        MacCready-flavoured choice for proposal 2)."""
        candidates = self.reachable(x, y, altitude, glide_ratio, reserve)
        if not candidates:
            return None
        return max(candidates, key=lambda th: self.score(th, now))
