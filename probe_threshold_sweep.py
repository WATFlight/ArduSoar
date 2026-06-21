"""PROBE_THRESHOLD sweep study.

Decrements the probe-mode confidence threshold (`PROBE_THRESHOLD`) by 0.02 per
step to try to make the glider probe more aggressively, runs one closed-loop
simulation per value, and renders a 6-panel animated GIF.

Finding: in this simplified model the threshold is *inert* — the confidence
metric (1/(1+mse)) is ~1 with a real signal and undefined otherwise, so it
rarely sits in the gated band. Vario noise is enabled here only so PROBE mode
is actually exercised and visible; the trajectories are still identical across
all thresholds. Path is coloured by mode: grey=cruise, amber=probe, green=thermal.

    python probe_threshold_sweep.py
"""

import os

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

from config import START_X, START_Y, WAYPOINT_X, WAYPOINT_Y, THERMAL_R
from simulator.simulation import run_simulation

THERMAL_X, THERMAL_Y = 230.0, 170.0   # off the y=x cruise line
SIM_TIME = 300.0
VARIO_NOISE = 1.3                      # so confidence is intermediate -> PROBE active
NOISE_SEED = 5
THRESHOLDS = [round(0.20 - 0.02 * i, 2) for i in range(6)]  # 0.20 .. 0.10
_MODE_COLOR = {"cruise": "#888780", "probe": "#BA7517", "thermal": "#0F6E56"}
_OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "output")


def _run_sweep():
    logs = []
    for th in THRESHOLDS:
        log = run_simulation(
            thermal_x=THERMAL_X, thermal_y=THERMAL_Y, sim_time=SIM_TIME,
            probe_threshold=th, vario_noise_std=VARIO_NOISE, noise_seed=NOISE_SEED,
        )
        logs.append(log)
        gain = log.h[-1] - log.h[0]
        probe_frac = sum(m == "probe" for m in log.mode) / len(log.mode)
        print(f"  probe_threshold={th}: altitude {gain:+.0f} m, time in probe {probe_frac*100:.0f}%")
    return logs


def render(filename: str = "probe_threshold_sweep.gif", stride: int = 20, fps: int = 18) -> str:
    print(f"Running PROBE_THRESHOLD sweep (vario noise={VARIO_NOISE}):")
    logs = _run_sweep()

    n_steps = len(logs[0].t)
    idx = list(range(0, n_steps, stride))
    if idx[-1] != n_steps - 1:
        idx.append(n_steps - 1)

    fig, axes = plt.subplots(2, 3, figsize=(13, 8.5))
    axes = axes.ravel()
    panels = []
    for ax, th, log in zip(axes, THRESHOLDS, logs):
        ax.plot([START_X, WAYPOINT_X], [START_Y, WAYPOINT_Y], ls=":", color="#888780", lw=1)
        ax.scatter([THERMAL_X], [THERMAL_Y], color="#D85A30", marker="*", s=130, zorder=5)
        ax.add_patch(plt.Circle((THERMAL_X, THERMAL_Y), THERMAL_R, color="#D85A30", fill=False, ls="--", alpha=0.5))
        ax.scatter([START_X], [START_Y], color="#1D9E75", s=25, zorder=5)
        ax.set_xlim(-30, 340)
        ax.set_ylim(-30, 340)
        ax.set_aspect("equal")
        ax.set_title(f"probe threshold = {th}", fontsize=11)
        ax.grid(alpha=0.25)
        scat = ax.scatter([], [], s=4)
        dot, = ax.plot([], [], marker="o", color="#185FA5", ms=6)
        txt = ax.text(0.03, 0.97, "", transform=ax.transAxes, va="top", fontsize=9)
        panels.append((log, scat, dot, txt))

    def update(frame_i):
        i = idx[frame_i]
        arts = []
        for log, scat, dot, txt in panels:
            xs = log.x[: i + 1]
            ys = log.y[: i + 1]
            cols = [_MODE_COLOR.get(m, "#185FA5") for m in log.mode[: i + 1]]
            scat.set_offsets(np.column_stack([xs, ys]))
            scat.set_color(cols)
            dot.set_data([log.x[i]], [log.y[i]])
            dot.set_color(_MODE_COLOR.get(log.mode[i], "#185FA5"))
            txt.set_text(f"t={log.t[i]:.0f}s  h={log.h[i]:.0f}m\n{log.mode[i]}")
            arts += [scat, dot, txt]
        return arts

    fig.suptitle(
        "PROBE_THRESHOLD sweep 0.20 → 0.10  —  trajectories identical (threshold is inert)\n"
        "path colour: grey=cruise, amber=probe, green=thermal",
        fontsize=13,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.94))

    anim = FuncAnimation(fig, update, frames=len(idx), blit=False)
    os.makedirs(_OUTPUT_DIR, exist_ok=True)
    out = os.path.abspath(os.path.join(_OUTPUT_DIR, filename))
    anim.save(out, writer=PillowWriter(fps=fps))
    plt.close(fig)
    return out


if __name__ == "__main__":
    out = render()
    print(f"Saved sweep animation to {out}")
