"""Monte Carlo driver (proposal §17/§19).

Runs many simulations with the thermal placed at a random offset from its
nominal position, then summarises how robustly the glider locates and climbs
it. This stress-tests the estimator + guidance the way the parent project's
``monte_carlo`` folder does.

    python -m monte_carlo.run_monte_carlo            # default 30 runs
    python -m monte_carlo.run_monte_carlo --n 60
"""

from __future__ import annotations

import argparse
import os

import numpy as np

from config import THERMAL_X, THERMAL_Y
from simulator.simulation import run_simulation
from monte_carlo.analysis import TrialResult, analyze, plot_results

_OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "..", "output")


def run_monte_carlo(
    n: int = 30,
    seed: int = 42,
    x_sigma: float = 70.0,
    y_sigma: float = 70.0,
    sim_time: float = 300.0,
) -> list[TrialResult]:
    """Run ``n`` trials with a Gaussian-random thermal-centre offset."""
    rng = np.random.default_rng(seed)
    results: list[TrialResult] = []
    for i in range(n):
        tx = THERMAL_X + rng.normal(0.0, x_sigma)
        ty = THERMAL_Y + rng.normal(0.0, y_sigma)
        log = run_simulation(thermal_x=tx, thermal_y=ty, sim_time=sim_time)
        results.append(analyze(log, tx, ty))
    return results


def _summary(results: list[TrialResult]) -> str:
    ok = [r for r in results if r.success]
    rate = 100.0 * len(ok) / len(results) if results else 0.0
    lines = [f"Ran {len(results)} trials — {len(ok)} succeeded ({rate:.0f}%)."]
    if ok:
        gains = np.array([r.altitude_gain_m for r in ok])
        errs = np.array([r.core_distance_error_m for r in ok])
        climbs = np.array([r.climb_rate_m_per_s for r in ok])
        lines.append(f"  mean altitude gain : {gains.mean():+.0f} m")
        lines.append(f"  mean climb rate    : {climbs.mean():.2f} m/s")
        lines.append(f"  mean core error    : {errs.mean():.1f} m")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="Monte Carlo thermal-offset analysis")
    parser.add_argument("--n", type=int, default=30, help="number of trials")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--sim-time", type=float, default=300.0)
    args = parser.parse_args()

    print(f"Running {args.n} Monte Carlo trials...")
    results = run_monte_carlo(n=args.n, seed=args.seed, sim_time=args.sim_time)
    print(_summary(results))

    out = plot_results(results, os.path.join(_OUTPUT_DIR, "monte_carlo_analysis.png"))
    print(f"Saved analysis to {out}")


if __name__ == "__main__":
    main()
