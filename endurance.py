"""Endurance soaring: no destination — stay aloft as long as possible.

Home is the ORIGIN. The glider greedily works thermals (which appear, disappear,
and MERGE) to stay up; whenever it has nothing to climb it heads home; and when
it finally can't stay up it glides back and lands at the origin.

    python endurance.py
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
from thermal_model.random_field import make_random_world
from thermal_model.lifecycle_thermal import LifecycleThermal
from thermal_model.merging_field import MergingField
from thermal_estimator.estimator import ThermalEstimator
from controller.state_machine import GuidanceStateMachine, GuidanceMode
from controller.l1_guidance import L1Guidance
from controller.circling_control import CirclingControl
from sensors.simulated import SimulatedSensorSuite, ground_truth_from_sim, SensorConfig, NoiseSpec
from navigation.thermal_prior import build_prior, BeliefMap
from navigation.decision import worth_climbing
from lifecycle_search import _search_schedule, _expanding_figure8
from thermal_model.random_field import make_uploaded_map

ORIGIN = (0.0, 0.0)
BOUNDS = (-2000.0, 2000.0, -2000.0, 2000.0)
START_H = 650.0
CLOUD_BASE = 780.0
LOW_ALT = 330.0
MIN_CLIMB = 0.3
RETURN_ALT = 230.0               # below this with nothing reachable -> go home
HOME_RANGE = 1400.0              # only work thermals within this of home (stay near)
LOITER_RADIUS = 250.0            # loiter over home when there's nothing to climb
MAX_TIME = 6000.0
INITIAL_COUNT = 16
SPAWN_RATE = 0.020
MERGE_DIST = 75.0
SEARCH_ENTER_RADIUS = 45.0
SEED = 4
_OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "output")


def _best_home_target(belief, state):
    """Best reachable candidate within HOME_RANGE of home (keeps us near base)."""
    near = [c for c in belief.active() if math.hypot(c.x - ORIGIN[0], c.y - ORIGIN[1]) <= HOME_RANGE]
    rng = max(0.0, state.h - 80.0) * 22.0
    reach = [c for c in near if math.hypot(c.x - state.x, c.y - state.y) <= rng]
    if not reach:
        return None
    return max(reach, key=lambda c: c.prob * c.strength_guess)


def _clean_cfg():
    cfg = SensorConfig()
    for f in ("accel", "gyro", "gps_pos", "gps_vel", "compass", "pitot",
              "baro_alt", "baro_vspeed", "temp", "humidity"):
        setattr(cfg, f, NoiseSpec(std=0.0))
    return cfg


def _build_field():
    base = make_random_world(BOUNDS, MAX_TIME, wind=(0.0, 0.0), seed=SEED,
                             initial_count=INITIAL_COUNT, spawn_rate=SPAWN_RATE,
                             grow_range=(90, 180), hold_range=(500, 1100), decay_range=(250, 450))
    # add close twins to a few thermals so we get visible merges
    rng = np.random.default_rng(SEED + 2)
    twins = []
    for th in rng.choice(base.thermals, size=min(4, len(base.thermals)), replace=False):
        twins.append(LifecycleThermal(
            th.x + float(rng.uniform(-45, 45)), th.y + float(rng.uniform(-45, 45)),
            float(rng.uniform(2.8, 4.0)), float(rng.uniform(45, 60)),
            th.birth + float(rng.uniform(-60, 60)), th.t_grow, th.t_hold, th.t_decay))
    base.thermals += twins
    return MergingField(base.thermals, merge_dist=MERGE_DIST)


def run():
    field = _build_field()
    uploaded = make_uploaded_map(field, upload_time=0.0, seed=SEED + 1,
                                 offset=(60.0, -40.0), rotation_deg=1.0, pos_noise=30.0)
    belief = BeliefMap(build_prior(uploaded, (0.0, 0.0), 0.0))

    state = GliderState(ORIGIN[0], ORIGIN[1], START_H, math.radians(20), AIRSPEED)
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
    going_home = False
    log = {k: [] for k in ("t", "x", "y", "h", "mode")}

    for k in range(int(MAX_TIME / DT)):
        t = k * DT
        lift = field.vertical_velocity(state.x, state.y, t)
        sink = glider.sink_rate()
        h_dot = glider.step(bank, lift, DT)
        if state.h < 0.0:
            break

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
            target = None
            searching = False
        prev_mode = mode

        if mode == GuidanceMode.THERMAL and active is not None:
            bank = circling.command(state, active)
        elif searching:
            search_t += DT
            if search_t > search_total:
                if target is not None:
                    belief.disconfirm(target)
                target = None
                searching = False
                bank = l1.bank_to_point(state, *ORIGIN)
            else:
                bank = _expanding_figure8(search_t, segs)
        else:
            # no thermal right now
            d_home = math.hypot(state.x - ORIGIN[0], state.y - ORIGIN[1])
            if target is None:
                target = _best_home_target(belief, state)
            if state.h < RETURN_ALT:
                going_home = True
                bank = l1.bank_to_point(state, *ORIGIN)        # low -> glide home to land
            elif target is None:
                # nothing to climb: loiter over home instead of flying off
                if d_home > LOITER_RADIUS:
                    bank = l1.bank_to_point(state, *ORIGIN)
                else:
                    bank = math.radians(22.0)                  # gentle circle over home
            elif math.hypot(target.x - state.x, target.y - state.y) <= SEARCH_ENTER_RADIUS:
                searching = True; search_t = 0.0
                bank = _expanding_figure8(0.0, segs)
            else:
                bank = l1.bank_to_point(state, target.x, target.y)

        for key, val in (("t", t), ("x", state.x), ("y", state.y), ("h", state.h), ("mode", mode.value)):
            log[key].append(val)

    flight_time = log["t"][-1]
    crash_dist = math.hypot(log["x"][-1] - ORIGIN[0], log["y"][-1] - ORIGIN[1])
    return field, belief, log, {"climbs": climbs, "flight_time": flight_time, "crash_dist": crash_dist}


def summarize(field, log, ev):
    print(f"endurance: stayed aloft {ev['flight_time']:.0f} s = {ev['flight_time']/60:.1f} min, "
          f"{ev['climbs']} climbs")
    print(f"peak {max(log['h']):.0f} m, min {min(log['h']):.0f} m")
    print(f"landed {ev['crash_dist']:.0f} m from the origin")


def render(field, log, filename="endurance_2d.gif", stride=40, fps=24):
    x = np.array(log["x"]); y = np.array(log["y"]); h = np.array(log["h"]); t = np.array(log["t"])
    mode = log["mode"]
    idx = list(range(0, len(x), stride)) + [len(x) - 1]
    mc = {"cruise": "#888780", "probe": "#BA7517", "thermal": "#0F6E56"}

    fig, (axp, axh) = plt.subplots(1, 2, figsize=(13, 6.2))
    axp.scatter([ORIGIN[0]], [ORIGIN[1]], color="#534AB7", marker="*", s=160, zorder=7, label="home (origin)")
    axp.set_xlim(BOUNDS[0] - 60, BOUNDS[1] + 60); axp.set_ylim(BOUNDS[2] - 60, BOUNDS[3] + 60)
    axp.set_aspect("equal"); axp.grid(alpha=0.25); axp.legend(fontsize=9, loc="upper left")
    axp.set_xlabel("x (m)"); axp.set_ylabel("y (m)")
    axp.set_title("merging field (big star = fused thermals); home = origin")
    th_scat = axp.scatter([], [], c=[], s=[], marker="*", zorder=5, cmap="autumn_r", vmin=0, vmax=5)
    trail, = axp.plot([], [], color="#185FA5", lw=0.9)
    dot, = axp.plot([], [], marker="o", ms=8, color="#185FA5")

    axh.plot(t, h, color="#185FA5", lw=1.0)
    axh.axhline(CLOUD_BASE, ls="--", color="#D85A30", lw=1)
    axh.axhline(0, ls="-", color="#A32D2D", lw=1)
    axh.set_xlabel("time (s)"); axh.set_ylabel("altitude (m)"); axh.grid(alpha=0.25)
    axh.set_title("altitude (endurance)")
    hdot, = axh.plot([], [], marker="o", ms=8, color="#185FA5")
    txt = axh.text(0.03, 0.95, "", transform=axh.transAxes, va="top", fontsize=11)

    def upd(fi):
        i = idx[fi]; ti = t[i]
        eff = field.effective(ti)
        pts = [(cx, cy) for cx, cy, W, R, n in eff if W > 0.1]
        cols = [W for cx, cy, W, R, n in eff if W > 0.1]
        sizes = [40 + 40 * W + 60 * (n - 1) for cx, cy, W, R, n in eff if W > 0.1]
        th_scat.set_offsets(pts if pts else np.empty((0, 2)))
        if pts:
            th_scat.set_array(np.array(cols)); th_scat.set_sizes(sizes)
        trail.set_data(x[: i + 1], y[: i + 1])
        dot.set_data([x[i]], [y[i]]); dot.set_color(mc.get(mode[i], "#185FA5"))
        hdot.set_data([t[i]], [h[i]]); hdot.set_color(mc.get(mode[i], "#185FA5"))
        txt.set_text(f"t={ti:.0f}s ({ti/60:.1f} min)  h={h[i]:.0f}m\nmode: {mode[i]}")
        return th_scat, trail, dot, hdot, txt

    fig.suptitle("Endurance: stay aloft as long as possible, land back at the origin", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    anim = FuncAnimation(fig, upd, frames=len(idx), blit=False)
    os.makedirs(_OUTPUT_DIR, exist_ok=True)
    out = os.path.abspath(os.path.join(_OUTPUT_DIR, filename))
    anim.save(out, writer=PillowWriter(fps=fps))
    plt.close(fig)
    return out


if __name__ == "__main__":
    field, belief, log, ev = run()
    summarize(field, log, ev)
    print("saved", render(field, log))
