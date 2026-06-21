"""Soaring a realistic random world (goals.md goal 1, end to end).

Random 2-D thermal field + stochastic birth-death + wind drift + a stale,
globally-offset uploaded map. The same guidance brain (prior-guided search,
bounded expanding search, capture, cloud-base hopping, belief decay,
dead-arrival, online map, value decision) flies it.

    python random_field_search.py
"""

import math
import os

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

from config import DT, AIRSPEED, START_HEADING, ESTIMATE_EVERY
from glider_model.glider import GliderState, SimpleGlider
from thermal_model.random_field import make_random_world, make_uploaded_map
from thermal_estimator.estimator import ThermalEstimator
from controller.state_machine import GuidanceStateMachine, GuidanceMode
from controller.l1_guidance import L1Guidance
from controller.circling_control import CirclingControl
from sensors.simulated import SimulatedSensorSuite, ground_truth_from_sim, SensorConfig, NoiseSpec
from navigation.thermal_prior import build_prior, BeliefMap
from navigation.thermal_map import ThermalMap
from navigation.decision import worth_climbing
from estimation.state_fusion import PassthroughFusion
from estimation.wind_estimator import SimpleWindEstimator
from lifecycle_search import _search_schedule, _expanding_figure8

# --- scenario ---
BOUNDS = (-300.0, 5200.0, -300.0, 4200.0)
START = (0.0, 0.0)
GOAL = (4600.0, 3400.0)
WIND = (0.0, 0.0)
START_H = 540.0
CLOUD_BASE = 740.0
SIM_TIME = 2800.0
INITIAL_COUNT = 22
SPAWN_RATE = 0.010
UPLOAD_OFFSET = (70.0, -45.0)
UPLOAD_ROTATION_DEG = 1.5
UPLOAD_POS_NOISE = 30.0
SEARCH_ENTER_RADIUS = 45.0
LOW_ALT = 380.0
MIN_CLIMB = 0.35
GOAL_RADIUS = 110.0
SEED = 10
_OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "output")


def _clean_cfg():
    cfg = SensorConfig()
    for f in ("accel", "gyro", "gps_pos", "gps_vel", "compass", "pitot",
              "baro_alt", "baro_vspeed", "temp", "humidity"):
        setattr(cfg, f, NoiseSpec(std=0.0))
    return cfg


def run():
    # realistic cumulus lifespans (~12-28 min) so a pre-flight map stays usable
    field = make_random_world(BOUNDS, SIM_TIME, wind=WIND, seed=SEED,
                              initial_count=INITIAL_COUNT, spawn_rate=SPAWN_RATE,
                              grow_range=(90, 180), hold_range=(500, 1100), decay_range=(250, 450))
    uploaded = make_uploaded_map(field, upload_time=0.0, seed=SEED + 1,
                                 offset=UPLOAD_OFFSET, rotation_deg=UPLOAD_ROTATION_DEG,
                                 pos_noise=UPLOAD_POS_NOISE)
    belief = BeliefMap(build_prior(uploaded, (0.0, 0.0), 0.0))
    live_map = ThermalMap(merge_dist=80.0)

    state = GliderState(START[0], START[1], START_H, START_HEADING, AIRSPEED)
    glider = SimpleGlider(state)
    suite = SimulatedSensorSuite(_clean_cfg(), seed=SEED)
    estimator = ThermalEstimator()
    sm = GuidanceStateMachine(cloud_base=CLOUD_BASE)
    l1 = L1Guidance()
    circling = CirclingControl(l1)
    fusion = PassthroughFusion()
    wind_est = SimpleWindEstimator(alpha=0.05)
    segs, search_total = _search_schedule(AIRSPEED)

    bank = 0.0
    estimate = None
    target = None
    searching = False
    search_t = 0.0
    prev_mode = None
    thermal_entry_h = None
    last_climb_pos = None
    skipping = False
    events = {"confirm": 0, "dead_arrival": 0, "opportunistic": 0, "skipped_weak": 0}
    log = {k: [] for k in ("t", "x", "y", "h", "mode")}

    for k in range(int(SIM_TIME / DT)):
        t = k * DT
        lift = field.vertical_velocity(state.x, state.y, t)
        sink = glider.sink_rate()
        h_dot = glider.step(bank, lift, DT)

        snap = suite.read(ground_truth_from_sim(t, state, h_dot, wind=WIND))
        vario = snap.baro.vertical_speed
        estimator.add_measurement(state.x, state.y, vario, sink)
        if k % ESTIMATE_EVERY == 0:
            estimate = estimator.estimate()
        # estimate the wind and advect the belief so candidates track the
        # drifting thermals (goal 1.3 / proposal 4).
        wind = wind_est.update(fusion.update(snap))
        estimator.set_wind(wind)        # fit/track thermals in the wind-moving frame
        belief.drift(wind, DT)
        belief.decay(DT)

        est_sm = estimate
        if estimate is not None and not worth_climbing(
                estimate, state.h, state.V, low_alt=LOW_ALT, min_climb_comfortable=MIN_CLIMB):
            est_sm = None
            if not skipping:
                events["skipped_weak"] += 1
                skipping = True
        else:
            skipping = False

        mode = sm.update(est_sm, altitude=state.h, position=(state.x, state.y))
        active = sm.active_estimate

        if mode == GuidanceMode.THERMAL and prev_mode != GuidanceMode.THERMAL:
            thermal_entry_h = state.h
        if prev_mode == GuidanceMode.THERMAL and mode == GuidanceMode.CRUISE:
            climbed = thermal_entry_h is not None and (state.h - thermal_entry_h) > 50.0
            if climbed and last_climb_pos is not None:
                live_map.add_or_update(last_climb_pos[0], last_climb_pos[1], 3.0, 50.0, t)
                if target is not None:
                    belief.confirm(target, last_climb_pos[0], last_climb_pos[1], 3.0)
                else:
                    events["opportunistic"] += 1
            elif target is not None:
                belief.disconfirm(target)
            target = None
            searching = False
        prev_mode = mode

        if mode == GuidanceMode.THERMAL and active is not None:
            bank = circling.command(state, active)
            last_climb_pos = (active.x_c, active.y_c)
            searching = False
        elif searching:
            search_t += DT
            if search_t > search_total:
                if target is not None:
                    belief.disconfirm(target)
                    live_map.mark_dead(target.x, target.y)
                    events["dead_arrival"] += 1
                target = None
                searching = False
                bank = l1.bank_to_point(state, GOAL[0], GOAL[1])
            else:
                bank = _expanding_figure8(search_t, segs)
        else:
            if target is None:
                target = belief.best_target(state.x, state.y, state.h, GOAL)
            if target is None:
                bank = l1.bank_to_point(state, GOAL[0], GOAL[1])
            elif math.hypot(target.x - state.x, target.y - state.y) <= SEARCH_ENTER_RADIUS:
                searching = True
                search_t = 0.0
                bank = _expanding_figure8(0.0, segs)
            else:
                bank = l1.bank_to_point(state, target.x, target.y)

        for key, val in (("t", t), ("x", state.x), ("y", state.y), ("h", state.h), ("mode", mode.value)):
            log[key].append(val)
        if math.hypot(state.x - GOAL[0], state.y - GOAL[1]) <= GOAL_RADIUS or state.h < 0:
            break

    events["confirm"] = sum(1 for c in belief.candidates if c.confirmed)
    return field, belief, log, events


def summarize(field, belief, log, events):
    reached = math.hypot(log["x"][-1] - GOAL[0], log["y"][-1] - GOAL[1]) <= GOAL_RADIUS
    print(f"world: random 2-D, {len(field.thermals)} thermals over the flight; "
          f"uploaded map: {len(belief.candidates)} (offset {UPLOAD_OFFSET}, rot {UPLOAD_ROTATION_DEG} deg)")
    print(f"confirmed: {events['confirm']}  opportunistic: {events['opportunistic']}  "
          f"dead-arrivals: {events['dead_arrival']}  weak-skipped: {events['skipped_weak']}")
    print(f"altitude: {log['h'][0]:.0f} -> {log['h'][-1]:.0f} m (peak {max(log['h']):.0f})")
    print(f"goal: {'REACHED' if reached else 'did not reach'} "
          f"(final {math.hypot(log['x'][-1]-GOAL[0], log['y'][-1]-GOAL[1]):.0f} m away)")
    return reached


def render(field, belief, log, filename="random_field_2d.gif", stride=45, fps=24):
    x = np.array(log["x"]); y = np.array(log["y"]); h = np.array(log["h"])
    t = np.array(log["t"]); mode = log["mode"]
    idx = list(range(0, len(x), stride)) + [len(x) - 1]
    mc = {"cruise": "#888780", "probe": "#BA7517", "thermal": "#0F6E56"}

    fig, (axp, axh) = plt.subplots(1, 2, figsize=(14, 6.5))
    # uploaded (offset) map candidates
    for c in belief.candidates:
        axp.scatter([c.x], [c.y], facecolors="none", edgecolors="#534AB7", s=70, lw=0.8, zorder=4)
    axp.scatter([START[0]], [START[1]], color="#1D9E75", s=45, zorder=7)
    axp.scatter([GOAL[0]], [GOAL[1]], color="#534AB7", marker="D", s=60, zorder=7)
    axp.set_xlim(BOUNDS[0] - 50, BOUNDS[1] + 50)
    axp.set_ylim(BOUNDS[2] - 50, BOUNDS[3] + 50)
    axp.set_aspect("equal"); axp.grid(alpha=0.25)
    axp.set_xlabel("x (m)"); axp.set_ylabel("y (m)")
    axp.set_title("random 2-D field (stars appear/disappear in place), open circles = offset uploaded map")
    th_scat = axp.scatter([], [], c=[], s=[], marker="*", zorder=5, cmap="autumn_r", vmin=0, vmax=4.5)
    trail, = axp.plot([], [], color="#185FA5", lw=1.0)
    pdot, = axp.plot([], [], marker="o", ms=8, color="#185FA5")

    axh.plot(t, h, color="#185FA5", lw=1.0)
    axh.axhline(CLOUD_BASE, ls="--", color="#D85A30", lw=1)
    axh.axhline(LOW_ALT, ls=":", color="#A32D2D", lw=1)
    axh.set_xlabel("time (s)"); axh.set_ylabel("altitude (m)"); axh.grid(alpha=0.25)
    axh.set_title("altitude")
    hdot, = axh.plot([], [], marker="o", ms=8, color="#185FA5")
    txt = axh.text(0.03, 0.95, "", transform=axh.transAxes, va="top", fontsize=11)

    def upd(fi):
        i = idx[fi]; ti = t[i]
        pts, cols, sizes = [], [], []
        for th in field.thermals:
            w = th.strength(ti)
            if w > 0.05:
                cx, cy = th.center(ti)
                pts.append((cx, cy)); cols.append(w); sizes.append(40 + 80 * w / 4.5)
        th_scat.set_offsets(pts if pts else np.empty((0, 2)))
        if pts:
            th_scat.set_array(np.array(cols)); th_scat.set_sizes(sizes)
        trail.set_data(x[: i + 1], y[: i + 1])
        pdot.set_data([x[i]], [y[i]]); pdot.set_color(mc.get(mode[i], "#185FA5"))
        hdot.set_data([t[i]], [h[i]]); hdot.set_color(mc.get(mode[i], "#185FA5"))
        txt.set_text(f"t={ti:.0f}s  h={h[i]:.0f}m\nmode: {mode[i]}")
        return th_scat, trail, pdot, hdot, txt

    fig.suptitle("Goal 1: realistic random world (2-D random appear/disappear + offset map)", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    anim = FuncAnimation(fig, upd, frames=len(idx), blit=False)
    os.makedirs(_OUTPUT_DIR, exist_ok=True)
    out = os.path.abspath(os.path.join(_OUTPUT_DIR, filename))
    anim.save(out, writer=PillowWriter(fps=fps))
    plt.close(fig)
    return out


if __name__ == "__main__":
    field, belief, log, events = run()
    summarize(field, belief, log, events)
    print("Saved", render(field, belief, log))
