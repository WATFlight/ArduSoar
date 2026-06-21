"""Energy depletion -> crash.

A few short-lived thermals and NO new ones: the sky 'dies'. The glider works
what's there, but once every thermal has dissipated it can find no more lift,
glides down, and finally runs out of altitude and crashes.

    python crash.py
"""

import math
import os

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

from config import DT, AIRSPEED, ESTIMATE_EVERY
from glider_model.glider import GliderState, SimpleGlider
from thermal_model.lifecycle_thermal import LifecycleThermal, TimeVaryingField
from thermal_model.random_field import make_uploaded_map
from thermal_estimator.estimator import ThermalEstimator
from controller.state_machine import GuidanceStateMachine, GuidanceMode
from controller.l1_guidance import L1Guidance
from controller.circling_control import CirclingControl
from sensors.simulated import SimulatedSensorSuite, ground_truth_from_sim, SensorConfig, NoiseSpec
from navigation.thermal_prior import build_prior, BeliefMap
from navigation.decision import worth_climbing
from lifecycle_search import _search_schedule, _expanding_figure8

ORIGIN = (0.0, 0.0)
START_H = 480.0
CLOUD_BASE = 620.0
LOW_ALT = 280.0
MIN_CLIMB = 0.3
SEARCH_ENTER_RADIUS = 45.0
MAX_TIME = 2000.0
SEED = 3
_OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "output")


def _clean_cfg():
    cfg = SensorConfig()
    for f in ("accel", "gyro", "gps_pos", "gps_vel", "compass", "pitot",
              "baro_alt", "baro_vspeed", "temp", "humidity"):
        setattr(cfg, f, NoiseSpec(std=0.0))
    return cfg


def _dying_field():
    """A handful of short-lived thermals, all gone within ~8 minutes, no new ones."""
    rng = np.random.default_rng(SEED)
    thermals = []
    for _ in range(6):
        x = float(rng.uniform(-850, 850))
        y = float(rng.uniform(-850, 850))
        thermals.append(LifecycleThermal(
            x, y, float(rng.uniform(3.0, 4.0)), 55.0,
            birth=-float(rng.uniform(0, 120)),
            t_grow=60.0, t_hold=float(rng.uniform(120, 220)), t_decay=100.0))
    return TimeVaryingField(thermals)


def run():
    field = _dying_field()
    uploaded = make_uploaded_map(field, upload_time=0.0, seed=SEED + 1,
                                 offset=(40.0, -25.0), pos_noise=25.0)
    belief = BeliefMap(build_prior(uploaded, (0.0, 0.0), 0.0))

    state = GliderState(ORIGIN[0], ORIGIN[1], START_H, math.radians(25), AIRSPEED)
    glider = SimpleGlider(state)
    suite = SimulatedSensorSuite(_clean_cfg(), seed=SEED)
    estimator = ThermalEstimator()
    sm = GuidanceStateMachine(cloud_base=CLOUD_BASE)
    l1 = L1Guidance()
    circling = CirclingControl(l1)
    segs, search_total = _search_schedule(AIRSPEED)

    bank = 0.0
    estimate = None
    target = None
    searching = False
    search_t = 0.0
    prev_mode = None
    climbs = 0
    sky_dead_t = None
    log = {k: [] for k in ("t", "x", "y", "h", "mode", "n_live")}

    for k in range(int(MAX_TIME / DT)):
        t = k * DT
        lift = field.vertical_velocity(state.x, state.y, t)
        sink = glider.sink_rate()
        h_dot = glider.step(bank, lift, DT)

        n_live = len(field.alive_thermals(t))
        if n_live == 0 and sky_dead_t is None:
            sky_dead_t = t

        snap = suite.read(ground_truth_from_sim(t, state, h_dot))
        estimator.add_measurement(state.x, state.y, snap.baro.vertical_speed, sink)
        if k % ESTIMATE_EVERY == 0:
            estimate = estimator.estimate()
        belief.decay(DT)

        est_sm = estimate if (estimate is not None and worth_climbing(
            estimate, state.h, state.V, low_alt=LOW_ALT, min_climb_comfortable=MIN_CLIMB)) else None
        mode = sm.update(est_sm, altitude=state.h, position=(state.x, state.y))
        active = sm.active_estimate

        if mode == GuidanceMode.THERMAL and prev_mode != GuidanceMode.THERMAL:
            climbs += 1
        if prev_mode == GuidanceMode.THERMAL and mode == GuidanceMode.CRUISE:
            target = None; searching = False
        prev_mode = mode

        if mode == GuidanceMode.THERMAL and active is not None:
            bank = circling.command(state, active)
        elif searching:
            search_t += DT
            if search_t > search_total:
                if target is not None:
                    belief.disconfirm(target)
                target = None; searching = False
                bank = l1.bank_to_point(state, *ORIGIN)
            else:
                bank = _expanding_figure8(search_t, segs)
        else:
            if target is None:
                target = belief.best_target(state.x, state.y, state.h, ORIGIN)
            if target is None:
                bank = l1.bank_to_point(state, *ORIGIN)   # no lift anywhere -> limp home
            elif math.hypot(target.x - state.x, target.y - state.y) <= SEARCH_ENTER_RADIUS:
                searching = True; search_t = 0.0
                bank = _expanding_figure8(0.0, segs)
            else:
                bank = l1.bank_to_point(state, target.x, target.y)

        crashed = state.h <= 0.0
        for key, val in (("t", t), ("x", state.x), ("y", state.y), ("h", max(state.h, 0.0)),
                         ("mode", mode.value), ("n_live", n_live)):
            log[key].append(val)
        if crashed:
            break

    info = {"climbs": climbs, "flight_time": log["t"][-1], "crashed": log["h"][-1] <= 0.0,
            "crash_xy": (log["x"][-1], log["y"][-1]), "sky_dead_t": sky_dead_t}
    return field, info, log


def summarize(info):
    print(f"climbs while the sky was alive: {info['climbs']}")
    if info["sky_dead_t"] is not None:
        print(f"last thermal died at t={info['sky_dead_t']:.0f} s ({info['sky_dead_t']/60:.1f} min)")
    if info["crashed"]:
        print(f"CRASHED at t={info['flight_time']:.0f} s ({info['flight_time']/60:.1f} min), "
              f"position ({info['crash_xy'][0]:.0f}, {info['crash_xy'][1]:.0f})")
    else:
        print("survived the whole window")


def render(field, info, log, filename="crash_2d.gif", stride=30, fps=24):
    x = np.array(log["x"]); y = np.array(log["y"]); h = np.array(log["h"]); t = np.array(log["t"])
    mode = log["mode"]
    idx = list(range(0, len(x), stride)) + [len(x) - 1]
    mc = {"cruise": "#888780", "probe": "#BA7517", "thermal": "#0F6E56"}

    fig, (axp, axh) = plt.subplots(1, 2, figsize=(13, 6.2))
    axp.scatter([ORIGIN[0]], [ORIGIN[1]], color="#534AB7", marker="s", s=60, zorder=7, label="home")
    allx = [th.x for th in field.thermals]; ally = [th.y for th in field.thermals]
    axp.set_xlim(min(min(allx), x.min()) - 150, max(max(allx), x.max()) + 150)
    axp.set_ylim(min(min(ally), y.min()) - 150, max(max(ally), y.max()) + 150)
    axp.set_aspect("equal"); axp.grid(alpha=0.25); axp.legend(fontsize=9, loc="upper left")
    axp.set_xlabel("x (m)"); axp.set_ylabel("y (m)")
    axp.set_title("thermals die off (no new ones) -> energy runs out")
    th_scat = axp.scatter([], [], c=[], s=[], marker="*", zorder=5, cmap="autumn_r", vmin=0, vmax=4.5)
    trail, = axp.plot([], [], color="#185FA5", lw=1.0)
    dot, = axp.plot([], [], marker="o", ms=8, color="#185FA5")
    crashx, = axp.plot([], [], marker="X", ms=16, color="#A32D2D", ls="none")

    axh.plot(t, h, color="#185FA5", lw=1.0)
    axh.axhline(CLOUD_BASE, ls="--", color="#D85A30", lw=1)
    axh.axhline(0, ls="-", color="#A32D2D", lw=1.2)
    if info["sky_dead_t"] is not None:
        axh.axvline(info["sky_dead_t"], ls=":", color="#444441", lw=1, label="last thermal dies")
        axh.legend(fontsize=9, loc="upper right")
    axh.set_xlabel("time (s)"); axh.set_ylabel("altitude (m)"); axh.grid(alpha=0.25)
    axh.set_title("altitude -> 0 (crash)")
    hdot, = axh.plot([], [], marker="o", ms=8, color="#185FA5")
    txt = axh.text(0.03, 0.95, "", transform=axh.transAxes, va="top", fontsize=11)

    def upd(fi):
        i = idx[fi]; ti = t[i]
        pts, cols, sizes = [], [], []
        for th in field.thermals:
            w = th.strength(ti)
            if w > 0.05:
                pts.append((th.x, th.y)); cols.append(w); sizes.append(40 + 70 * w / 4.5)
        th_scat.set_offsets(pts if pts else np.empty((0, 2)))
        if pts:
            th_scat.set_array(np.array(cols)); th_scat.set_sizes(sizes)
        trail.set_data(x[: i + 1], y[: i + 1])
        dot.set_data([x[i]], [y[i]]); dot.set_color(mc.get(mode[i], "#185FA5"))
        hdot.set_data([t[i]], [h[i]]); hdot.set_color(mc.get(mode[i], "#185FA5"))
        if fi == len(idx) - 1 and info["crashed"]:
            crashx.set_data([x[i]], [y[i]])
            txt.set_text(f"t={ti:.0f}s ({ti/60:.1f} min)  h={h[i]:.0f}m\nCRASHED — energy depleted")
        else:
            crashx.set_data([], [])
            txt.set_text(f"t={ti:.0f}s ({ti/60:.1f} min)  h={h[i]:.0f}m\nmode: {mode[i]}  live thermals: {log['n_live'][i]}")
        return th_scat, trail, dot, hdot, crashx, txt

    fig.suptitle("Energy depletion: the sky dies, the glider sinks and crashes", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    anim = FuncAnimation(fig, upd, frames=len(idx), blit=False)
    os.makedirs(_OUTPUT_DIR, exist_ok=True)
    out = os.path.abspath(os.path.join(_OUTPUT_DIR, filename))
    anim.save(out, writer=PillowWriter(fps=fps))
    plt.close(fig)
    return out


if __name__ == "__main__":
    field, info, log = run()
    summarize(info)
    print("saved", render(field, info, log))
