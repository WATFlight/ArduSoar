"""Tests for the Monte Carlo robustness analysis."""

import math

from monte_carlo.run_monte_carlo import run_monte_carlo
from monte_carlo.analysis import analyze
from simulator.simulation import run_simulation
from config import THERMAL_X, THERMAL_Y


def test_nominal_trial_succeeds():
    """With the thermal at its nominal place, the run should climb."""
    log = run_simulation(sim_time=300.0)
    res = analyze(log, THERMAL_X, THERMAL_Y)
    assert res.success
    assert res.altitude_gain_m > 0.0
    # The estimator should land within a few tens of metres of the true core.
    assert res.core_distance_error_m < 30.0


def test_monte_carlo_mostly_succeeds():
    """A small batch with random offsets should mostly find and climb the thermal."""
    results = run_monte_carlo(n=6, seed=1, x_sigma=20.0, y_sigma=20.0, sim_time=300.0)
    assert len(results) == 6
    success_rate = sum(r.success for r in results) / len(results)
    assert success_rate >= 0.5

    # Every successful trial should report finite, sensible steady-state metrics.
    for r in results:
        if r.success:
            assert r.altitude_gain_m > 0.0
            assert not math.isnan(r.core_distance_error_m)
            assert r.est_w_max_m_per_s > 0.0
