"""Plotting of simulation results (Step 11 of the proposal).

Produces a single figure with: the 2D flight path, altitude vs time, the flight
mode vs time, and net climb vs time. Saves to ``output/`` so it works without a
display.
"""

import os

import matplotlib

matplotlib.use("Agg")  # headless backend; no GUI required
import matplotlib.pyplot as plt

from config import THERMAL_X, THERMAL_Y, THERMAL_R, WAYPOINT_X, WAYPOINT_Y
from simulator.simulation import SimLog

_MODE_TO_Y = {"cruise": 0, "probe": 1, "thermal": 2}
_OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "..", "output")


def plot_results(log: SimLog, filename: str = "simulation.png") -> str:
    fig, axes = plt.subplots(2, 2, figsize=(13, 10))

    _plot_path(axes[0, 0], log)
    _plot_altitude(axes[0, 1], log)
    _plot_mode(axes[1, 0], log)
    _plot_net_climb(axes[1, 1], log)

    fig.tight_layout()
    os.makedirs(_OUTPUT_DIR, exist_ok=True)
    out_path = os.path.abspath(os.path.join(_OUTPUT_DIR, filename))
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    return out_path


def _plot_path(ax, log: SimLog):
    ax.plot(log.x, log.y, lw=1.0, color="#185FA5", label="flight path")
    ax.scatter([log.x[0]], [log.y[0]], color="#1D9E75", s=40, zorder=5, label="start")

    # True thermal core and radius.
    ax.scatter([THERMAL_X], [THERMAL_Y], color="#D85A30", marker="*", s=160, zorder=5, label="true core")
    circ = plt.Circle((THERMAL_X, THERMAL_Y), THERMAL_R, color="#D85A30", fill=False, ls="--", alpha=0.5)
    ax.add_patch(circ)

    # Last available estimated core.
    est = [(x, y) for x, y in zip(log.est_x_c, log.est_y_c) if x is not None]
    if est:
        ex, ey = est[-1]
        ax.scatter([ex], [ey], color="#534AB7", marker="x", s=80, zorder=6, label="last est. core")

    ax.scatter([WAYPOINT_X], [WAYPOINT_Y], color="#888780", marker="s", s=40, label="waypoint")
    ax.set_aspect("equal", "datalim")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Flight path")
    ax.legend(fontsize=8, loc="best")
    ax.grid(alpha=0.3)


def _plot_altitude(ax, log: SimLog):
    ax.plot(log.t, log.h, color="#185FA5")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("altitude (m)")
    ax.set_title("Altitude vs time")
    ax.grid(alpha=0.3)


def _plot_mode(ax, log: SimLog):
    ys = [_MODE_TO_Y[m] for m in log.mode]
    ax.plot(log.t, ys, color="#534AB7", drawstyle="steps-post")
    ax.set_yticks([0, 1, 2])
    ax.set_yticklabels(["cruise", "probe", "thermal"])
    ax.set_xlabel("time (s)")
    ax.set_title("Flight mode vs time")
    ax.grid(alpha=0.3)


def _plot_net_climb(ax, log: SimLog):
    ax.plot(log.t, log.h_dot, color="#0F6E56", lw=0.9)
    ax.axhline(0.0, color="#888780", ls="--", lw=0.8)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("net climb h_dot (m/s)")
    ax.set_title("Net climb vs time")
    ax.grid(alpha=0.3)
