"""Cross-country soaring scenario (Phase A): multiple thermals, a far goal.

A corridor of thermals is placed along the start->goal route. The glider cruises
toward the goal with the figure-8 search, captures each thermal it sweeps over,
climbs to cloud base, leaves it (and is barred from re-entering *that* thermal),
glides on, and hops to the next one — a basic thermal-hopping cross-country.

    python cross_country.py
"""

import math
import os

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

from config import START_X, START_Y, THERMAL_R
from simulator.simulation import run_simulation
from thermal_model.thermal_field import make_corridor_field

# --- scenario ---
START = (START_X, START_Y)
GOAL = (5000.0, 5000.0)
N_THERMALS = 8
PERP_JITTER = 90.0          # max perpendicular offset of thermals from the route
CLOUD_BASE = 700.0
START_H = 300.0
SIM_TIME = 2800.0
GOAL_RADIUS = 80.0          # counts as "reached" within this distance
FIELD_SEED = 7

_MODE_COLOR = {"cruise": "#888780", "probe": "#BA7517", "thermal": "#0F6E56"}
_OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "output")


def run():
    field = make_corridor_field(START, GOAL, N_THERMALS, perp_jitter=PERP_JITTER, seed=FIELD_SEED)
    log = run_simulation(
        field=field,
        start_h=START_H,
        sim_time=SIM_TIME,
        cloud_base=CLOUD_BASE,
        waypoint_x=GOAL[0],
        waypoint_y=GOAL[1],
        probe_threshold=0.1,
    )
    return field, log


def _summarize(field, log):
    # number of separate climbs = cruise/probe -> thermal transitions
    climbs = sum(
        1
        for a, b in zip(log.mode, log.mode[1:])
        if a != "thermal" and b == "thermal"
    )
    dmin = min(math.hypot(x - GOAL[0], y - GOAL[1]) for x, y in zip(log.x, log.y))
    reached = dmin <= GOAL_RADIUS
    print(f"thermals placed : {len(field.thermals)}")
    print(f"altitude        : {log.h[0]:.0f} -> {log.h[-1]:.0f} m  (peak {max(log.h):.0f})")
    print(f"separate climbs : {climbs}")
    print(f"closest to goal : {dmin:.0f} m  -> {'REACHED' if reached else 'did not reach'}")
    return climbs, reached


def render(field, log, filename="cross_country_2d.gif", stride=40, fps=24):
    x = np.array(log.x); y = np.array(log.y); h = np.array(log.h); t = np.array(log.t)
    mode = log.mode
    idx = list(range(0, len(x), stride))
    if idx[-1] != len(x) - 1:
        idx.append(len(x) - 1)

    fig, (axp, axh) = plt.subplots(1, 2, figsize=(14, 6.5))

    # path panel with all thermals + goal
    for th in field.thermals:
        axp.scatter([th.x_c], [th.y_c], color="#D85A30", marker="*", s=120, zorder=5)
        axp.add_patch(plt.Circle((th.x_c, th.y_c), th.R_th, color="#D85A30", fill=False, ls="--", alpha=0.4))
    axp.scatter([START[0]], [START[1]], color="#1D9E75", s=45, zorder=6, label="start")
    axp.scatter([GOAL[0]], [GOAL[1]], color="#534AB7", marker="D", s=55, zorder=6, label="goal")
    axp.plot([START[0], GOAL[0]], [START[1], GOAL[1]], ls=":", color="#888780", lw=1)
    axp.set_xlim(x.min() - 60, max(x.max(), GOAL[0]) + 60)
    axp.set_ylim(y.min() - 60, max(y.max(), GOAL[1]) + 60)
    axp.set_aspect("equal"); axp.grid(alpha=0.25); axp.legend(fontsize=9, loc="upper left")
    axp.set_xlabel("x (m)"); axp.set_ylabel("y (m)"); axp.set_title("Cross-country flight path")
    trail, = axp.plot([], [], color="#185FA5", lw=1.0)
    pdot, = axp.plot([], [], marker="o", ms=8, color="#185FA5")

    # altitude panel
    axh.plot(t, h, color="#185FA5", lw=1.0)
    axh.axhline(CLOUD_BASE, ls="--", color="#D85A30", lw=1, label=f"cloud base {CLOUD_BASE:.0f} m")
    axh.set_xlabel("time (s)"); axh.set_ylabel("altitude (m)"); axh.grid(alpha=0.25)
    axh.legend(fontsize=9, loc="lower left"); axh.set_title("Altitude vs time (thermal hopping)")
    hdot, = axh.plot([], [], marker="o", ms=8, color="#185FA5")
    mtxt = axh.text(0.03, 0.95, "", transform=axh.transAxes, va="top", fontsize=11)

    def upd(fi):
        i = idx[fi]
        trail.set_data(x[: i + 1], y[: i + 1])
        pdot.set_data([x[i]], [y[i]]); pdot.set_color(_MODE_COLOR.get(mode[i], "#185FA5"))
        hdot.set_data([t[i]], [h[i]]); hdot.set_color(_MODE_COLOR.get(mode[i], "#185FA5"))
        mtxt.set_text(f"t={t[i]:.0f}s  h={h[i]:.0f}m\nmode: {mode[i]}")
        return trail, pdot, hdot, mtxt

    fig.suptitle("Cross-country: hop thermals along the corridor to the goal", fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    anim = FuncAnimation(fig, upd, frames=len(idx), blit=False)
    os.makedirs(_OUTPUT_DIR, exist_ok=True)
    out = os.path.abspath(os.path.join(_OUTPUT_DIR, filename))
    anim.save(out, writer=PillowWriter(fps=fps))
    plt.close(fig)
    return out


if __name__ == "__main__":
    field, log = run()
    _summarize(field, log)
    out = render(field, log)
    print(f"Saved cross-country animation to {out}")
