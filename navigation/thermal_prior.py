"""Prior-guided thermal belief (the "upload a thermal map + wind before flight"
strategy).

Before flight you upload candidate thermal *source* locations (a thermal map)
and the wind. Thermals drift downwind, so the predicted *current* position of
each candidate is the source shifted downwind (proposal 4). Each candidate
carries a probability. In flight the glider flies to the best candidate, and
its own measurements confirm (found lift) or disconfirm (searched, nothing)
each one — a simple Bayesian update over a fixed candidate set.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class CandidatePoint:
    x: float                 # predicted current position (source + wind drift)
    y: float
    prob: float              # belief this is a real, usable thermal (0..1)
    strength_guess: float
    visited: bool = False    # we have searched here
    confirmed: bool = False   # we found and used lift here


def build_prior(uploaded_sources, wind, drift_distance: float = 0.0) -> list:
    """Turn uploaded source points into wind-drifted candidates.

    ``uploaded_sources``: list of (x, y, strength_guess[, prob]).
    ``wind``: object with .wx, .wy (or a (wx, wy) tuple); thermals drift the way
    the wind blows. ``drift_distance``: metres to shift downwind.
    """
    wx, wy = (wind.wx, wind.wy) if hasattr(wind, "wx") else (wind[0], wind[1])
    speed = math.hypot(wx, wy)
    ux, uy = (wx / speed, wy / speed) if speed > 1e-6 else (0.0, 0.0)
    cands = []
    for s in uploaded_sources:
        x, y, strength = s[0], s[1], s[2]
        prob = s[3] if len(s) > 3 else 0.6
        cands.append(CandidatePoint(
            x=x + ux * drift_distance,
            y=y + uy * drift_distance,
            prob=prob,
            strength_guess=strength,
        ))
    return cands


class BeliefMap:
    def __init__(self, candidates: list, min_prob: float = 0.12):
        self.candidates = candidates
        self.min_prob = min_prob

    def active(self):
        """Candidates still worth considering (not used up, not ruled out)."""
        return [c for c in self.candidates if c.prob >= self.min_prob and not c.confirmed]

    def best_target(self, x: float, y: float, altitude: float, goal,
                    glide_ratio: float = 22.0, reserve: float = 80.0) -> CandidatePoint | None:
        """Pick the best reachable candidate, preferring ones toward the goal."""
        usable = max(0.0, altitude - reserve)
        rng = usable * glide_ratio
        reach = [c for c in self.active() if math.hypot(c.x - x, c.y - y) <= rng]
        if not reach:
            return None
        d_goal_now = math.hypot(goal[0] - x, goal[1] - y)
        ahead = [c for c in reach if math.hypot(goal[0] - c.x, goal[1] - c.y) < d_goal_now]
        pool = ahead or reach
        # score: likely + strong + makes progress toward the goal
        def score(c):
            progress = d_goal_now - math.hypot(goal[0] - c.x, goal[1] - c.y)
            return c.prob * c.strength_guess + 0.002 * progress
        return max(pool, key=score)

    def confirm(self, c: CandidatePoint, x: float, y: float, strength: float) -> None:
        c.confirmed = True
        c.visited = True
        c.prob = min(1.0, c.prob + 0.4)
        c.x, c.y, c.strength_guess = x, y, strength   # refine with the real fix

    def disconfirm(self, c: CandidatePoint) -> None:
        c.visited = True
        c.prob *= 0.1                                  # searched, found nothing

    def drift(self, wind, dt: float) -> None:
        """Advect the (unconfirmed) candidates downwind so they track the
        drifting thermals — the map snapshot is only valid at upload time."""
        for c in self.candidates:
            if not c.confirmed:
                c.x += wind.wx * dt
                c.y += wind.wy * dt

    def decay(self, dt: float, tau: float = 600.0) -> None:
        """The uploaded map ages: unconfirmed candidates get less trustworthy as
        the flight goes on (a thermal predicted from a stale map may be gone)."""
        f = math.exp(-dt / tau)
        for c in self.candidates:
            if not c.confirmed:
                c.prob *= f
