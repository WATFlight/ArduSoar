"""Energy ('fuel') monitor for the explore mission.

For an unpowered glider the fuel is altitude -> glide range. Before committing to
a thermal the glider checks it can still glide home, and when the margin is low it
prefers thermals CLOSER TO HOME so working them does not strand it.
"""

import math
from dataclasses import dataclass

import explore


@dataclass
class _State:
    x: float
    y: float
    h: float


@dataclass
class _Cand:
    x: float
    y: float


def test_glide_range_and_home_margin():
    st = _State(3000.0, 0.0, 275.0)
    rng = explore._glide_range(st.h)               # (275-130)*22 = 3190 m
    assert abs(rng - (275.0 - explore.HOME_RESERVE) * explore.GLIDE_RATIO) < 1e-6
    assert abs(explore._home_margin(st) - (rng - 3000.0)) < 1e-6   # minus distance home


def test_low_fuel_prefers_thermal_near_home():
    """Low margin (far out, low alt): pick the home-ward thermal, not the closest."""
    st = _State(3000.0, 0.0, 275.0)                # margin ~190 m < ENERGY_MARGIN
    assert explore._home_margin(st) < explore.ENERGY_MARGIN
    cands = [_Cand(100.0, 0.0),                    # 0: near home (far from glider)
             _Cand(3100.0, 0.0)]                   # 1: next to glider (far from home)
    assert explore._select_target(st, cands, [0, 1]) == 0


def test_healthy_fuel_takes_nearest():
    """Plenty of margin (high alt): just take the nearest pending point."""
    st = _State(3000.0, 0.0, 900.0)                # margin ~14 km >> ENERGY_MARGIN
    assert explore._home_margin(st) > explore.ENERGY_MARGIN
    cands = [_Cand(100.0, 0.0),                    # far from glider
             _Cand(3100.0, 0.0)]                   # nearest to glider
    assert explore._select_target(st, cands, [0, 1]) == 1


def test_unreachable_round_trip_excluded_when_possible():
    """A point we cannot reach AND still get home from is avoided if a safe one exists."""
    st = _State(0.0, 0.0, 275.0)                   # range ~3190 m
    cands = [_Cand(1200.0, 0.0),                   # out 1200 + back 1200 = 2400 <= range (safe)
             _Cand(3000.0, 0.0)]                   # out 3000 + back 3000 = 6000 > range (unsafe)
    assert explore._select_target(st, cands, [0, 1]) == 0
