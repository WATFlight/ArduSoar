"""Metrics and plotting for the Monte Carlo runs.

Mirrors the analysis in the parent project's ``monte_carlo/`` folder, but works
on the in-memory ``SimLog`` returned by each simulation instead of JSONL files.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from simulator.simulation import SimLog

# How many of the final timesteps define "steady state".
STEADY_STATE_SAMPLES = 100
# Minimum steady-state climb (m/s) for a run to count as a success.
SUCCESS_CLIMB_THRESHOLD = 0.3


@dataclass
class TrialResult:
    thermal_x: float
    thermal_y: float
    core_distance_error_m: float        # |estimated core - true core| at steady state
    est_radius_m: float
    est_w_max_m_per_s: float
    climb_rate_m_per_s: float           # mean net climb at steady state
    altitude_gain_m: float              # final altitude minus start altitude
    time_to_thermal_s: float | None     # first time the mode became THERMAL
    success: bool


def analyze(log: SimLog, thermal_x: float, thermal_y: float) -> TrialResult:
    k = min(STEADY_STATE_SAMPLES, len(log.t))
    tail = slice(len(log.t) - k, len(log.t))

    # Steady-state estimate quality (only over samples where an estimate exists).
    dists, radii, w_maxs = [], [], []
    for ex, ey, er, ew in zip(
        log.est_x_c[tail], log.est_y_c[tail], log.est_R_th[tail], log.est_W_0[tail]
    ):
        if ex is None:
            continue
        dists.append(math.hypot(ex - thermal_x, ey - thermal_y))
        radii.append(er)
        w_maxs.append(ew)

    climb_rate = float(np.mean(log.h_dot[tail]))
    altitude_gain = log.h[-1] - log.h[0]

    time_to_thermal = None
    for t, m in zip(log.t, log.mode):
        if m == "thermal":
            time_to_thermal = t
            break

    reached_thermal = "thermal" in log.mode
    success = reached_thermal and climb_rate > SUCCESS_CLIMB_THRESHOLD

    return TrialResult(
        thermal_x=thermal_x,
        thermal_y=thermal_y,
        core_distance_error_m=float(np.mean(dists)) if dists else float("nan"),
        est_radius_m=float(np.mean(radii)) if radii else float("nan"),
        est_w_max_m_per_s=float(np.mean(w_maxs)) if w_maxs else float("nan"),
        climb_rate_m_per_s=climb_rate,
        altitude_gain_m=altitude_gain,
        time_to_thermal_s=time_to_thermal,
        success=success,
    )


# --- plotting --------------------------------------------------------------
def _hist(ax, data, title, xlabel):
    arr = np.asarray([d for d in data if d is not None and not math.isnan(d)])
    if arr.size == 0:
        ax.text(0.5, 0.5, "No data", ha="center", va="center")
        ax.set_title(title)
        ax.set_xlabel(xlabel)
        ax.set_yticks([])
        return
    ax.hist(arr, bins="auto", alpha=0.75, edgecolor="black", color="#185FA5")
    mu, med = float(np.mean(arr)), float(np.median(arr))
    ax.axvline(mu, ls="--", lw=1.5, color="#D85A30", label=f"mean = {mu:.3g}")
    ax.axvline(med, ls="-.", lw=1.5, color="#0F6E56", label=f"median = {med:.3g}")
    ax.set_title(f"{title}  (n={arr.size})")
    ax.set_xlabel(xlabel)
    ax.set_ylabel("count")
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8)


def _bar_success(ax, n_success, n_fail):
    total = n_success + n_fail
    ax.bar(["success", "fail"], [n_success, n_fail], color=["#1D9E75", "#D85A30"], edgecolor="black", alpha=0.85)
    for i, v in enumerate([n_success, n_fail]):
        pct = 100.0 * v / total if total else 0.0
        ax.text(i, v, f"{v} ({pct:.0f}%)", ha="center", va="bottom", fontsize=9)
    ax.set_title(f"Run outcomes  (total={total})")
    ax.set_ylabel("count")
    ax.grid(axis="y", alpha=0.25)


def plot_results(results: list[TrialResult], save_path: str) -> str:
    successful = [r for r in results if r.success]

    panels = [
        ([r.core_distance_error_m for r in successful], "Core distance error", "error (m)"),
        ([r.est_radius_m for r in successful], "Estimated radius", "R_th (m)"),
        ([r.est_w_max_m_per_s for r in successful], "Estimated strength", "W_0 (m/s)"),
        ([r.climb_rate_m_per_s for r in successful], "Steady-state climb", "h_dot (m/s)"),
        ([r.time_to_thermal_s for r in successful], "Time to circling", "time (s)"),
    ]

    n = len(panels) + 1
    cols = 3
    rows = math.ceil(n / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(5 * cols, 4 * rows))
    axes = np.ravel(axes)

    for i, (data, title, xlabel) in enumerate(panels):
        _hist(axes[i], data, title, xlabel)
    _bar_success(axes[len(panels)], len(successful), len(results) - len(successful))
    for j in range(n, len(axes)):
        axes[j].axis("off")

    fig.suptitle("Thermal-offset Monte Carlo analysis", fontsize=14)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    fig.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return os.path.abspath(save_path)
