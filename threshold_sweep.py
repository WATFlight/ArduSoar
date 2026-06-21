"""Detection-threshold sweep study.

Decrements the variometer lift-detection threshold (`DETECT_LIFT_THRESHOLD`) by
0.1 m/s per step, runs one simulation at each value with the thermal placed
moderately off the cruise line, and renders a single 6-panel animated GIF so the
flight behaviour can be compared side by side.

    python threshold_sweep.py
"""

import os

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

from config import START_X, START_Y, WAYPOINT_X, WAYPOINT_Y, THERMAL_R
from simulator.simulation import run_simulation

# Thermal sits off the y=x cruise line; the glider only grazes its edge.
THERMAL_X, THERMAL_Y = 240.0, 160.0
SIM_TIME = 300.0
THRESHOLDS = [round(0.6 - 0.1 * i, 1) for i in range(6)]  # 0.6 .. 0.1
_MODE_COLOR = {"cruise": "#888780", "probe": "#BA7517", "thermal": "#0F6E56"}
_OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "output")


def _run_sweep():
    logs = []
    for th in THRESHOLDS:
        log = run_simulation(
            thermal_x=THERMAL_X, thermal_y=THERMAL_Y, sim_time=SIM_TIME, detect_threshold=th
        )
        logs.append(log)
        gain = log.h[-1] - log.h[0]
        modes = "+".join(sorted(set(log.mode)))
        print(f"  detect_threshold={th}: altitude {gain:+.0f} m, modes={modes}")
    return logs


def render(filename: str = "threshold_sweep.gif", stride: int = 20, fps: int = 18) -> str:
    print(f"Running detection-threshold sweep at thermal ({THERMAL_X:.0f}, {THERMAL_Y:.0f}):")
    logs = _run_sweep()

    n_steps = len(logs[0].t)
    idx = list(range(0, n_steps, stride))
    if idx[-1] != n_steps - 1:
        idx.append(n_steps - 1)

    fig, axes = plt.subplots(2, 3, figsize=(13, 8.5))
    axes = axes.ravel()
    trails, dots, txts = [], [], []

    for ax, th, log in zip(axes, THRESHOLDS, logs):
        ax.plot([START_X, WAYPOINT_X], [START_Y, WAYPOINT_Y], ls=":", color="#888780", lw=1)
        ax.scatter([THERMAL_X], [THERMAL_Y], color="#D85A30", marker="*", s=130, zorder=5)
        ax.add_patch(plt.Circle((THERMAL_X, THERMAL_Y), THERMAL_R, color="#D85A30", fill=False, ls="--", alpha=0.5))
        ax.scatter([START_X], [START_Y], color="#1D9E75", s=25, zorder=5)
        ax.set_xlim(-30, 320)
        ax.set_ylim(-30, 320)
        ax.set_aspect("equal")
        ax.set_title(f"detect threshold = {th} m/s", fontsize=11)
        ax.grid(alpha=0.25)
        trail, = ax.plot([], [], color="#185FA5", lw=1.2)
        dot, = ax.plot([], [], marker="o", color="#185FA5", ms=6)
        txt = ax.text(0.03, 0.96, "", transform=ax.transAxes, va="top", fontsize=9)
        trails.append(trail)
        dots.append(dot)
        txts.append(txt)

    def update(frame_i):
        i = idx[frame_i]
        artists = []
        for log, trail, dot, txt in zip(logs, trails, dots, txts):
            trail.set_data(log.x[: i + 1], log.y[: i + 1])
            dot.set_data([log.x[i]], [log.y[i]])
            dot.set_color(_MODE_COLOR.get(log.mode[i], "#185FA5"))
            txt.set_text(f"t={log.t[i]:.0f}s  h={log.h[i]:.0f}m\n{log.mode[i]}")
            artists += [trail, dot, txt]
        return artists

    fig.suptitle("Variometer detection-threshold sweep (thermal off the cruise line)", fontsize=14)
    fig.tight_layout(rect=(0, 0, 1, 0.97))

    anim = FuncAnimation(fig, update, frames=len(idx), blit=False)
    os.makedirs(_OUTPUT_DIR, exist_ok=True)
    out = os.path.abspath(os.path.join(_OUTPUT_DIR, filename))
    anim.save(out, writer=PillowWriter(fps=fps))
    plt.close(fig)
    return out


if __name__ == "__main__":
    out = render()
    print(f"Saved sweep animation to {out}")
