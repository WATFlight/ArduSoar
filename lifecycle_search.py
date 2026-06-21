"""Soaring in a CHANGING world — all four lifecycle proposals combined.

  (1) time-varying thermals: they grow / hold / weaken / die, and some are born
      mid-flight (not on the uploaded map);
  (2) belief decay + dead-arrival: the uploaded map ages; arriving at a predicted
      thermal that has died -> disconfirm and move on;
  (3) online map update + opportunistic capture: confirm thermals we use, drop
      ones we find dead, and grab new thermals we stumble onto;
  (4) value decision (MacCready-ish): skip a weak/dying thermal when we are high,
      take anything when we are low.

Renders a video where the thermals visibly pulse in and out as the glider adapts.

    python lifecycle_search.py
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
from thermal_model.lifecycle_thermal import make_lifecycle_corridor
from thermal_estimator.estimator import ThermalEstimator
from controller.state_machine import GuidanceStateMachine, GuidanceMode
from controller.l1_guidance import L1Guidance
from controller.circling_control import CirclingControl
from sensors.simulated import SimulatedSensorSuite, ground_truth_from_sim, SensorConfig, NoiseSpec
from navigation.thermal_prior import build_prior, BeliefMap
from navigation.thermal_map import ThermalMap
from navigation.contact_detector import ContactDetector
from navigation.decision import worth_climbing
from estimation.wind_estimator import Wind

# --- scenario ---
START = (0.0, 0.0)
GOAL = (7000.0, 5000.0)
START_H = 480.0
CLOUD_BASE = 720.0
SIM_TIME = 2600.0
N_THERMALS = 13
BORN_FRACTION = 0.62            # rest are born mid-flight (not on the uploaded map)
PRIOR_POS_SIGMA = 45.0
SEARCH_ENTER_RADIUS = 45.0
SEARCH_BANKS_DEG = (35.0, 28.0, 22.0)
LOW_ALT = 380.0                 # below this, take any lift (survival)
MIN_CLIMB = 0.35                # above LOW_ALT, only circle thermals better than this
WIND = (2.0, 1.0)               # thermals now DRIFT downwind
GOAL_RADIUS = 100.0
SEED = 11
_OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "output")


def _clean_cfg():
    cfg = SensorConfig()
    for f in ("accel", "gyro", "gps_pos", "gps_vel", "compass", "pitot",
              "baro_alt", "baro_vspeed", "temp", "humidity"):
        setattr(cfg, f, NoiseSpec(std=0.0))
    return cfg


def _search_schedule(V, banks=SEARCH_BANKS_DEG):
    segs, cum = [], 0.0
    for d in banks:
        phi = math.radians(d)
        t_loop = 2.0 * math.pi / (9.81 * math.tan(phi) / V)
        segs.append((cum, t_loop, phi))
        cum += 2.0 * t_loop
    return segs, cum


def _expanding_figure8(te, segs):
    for start, t_loop, phi in segs:
        if te < start + 2.0 * t_loop:
            return -phi if (te - start) < t_loop else phi
    return 0.0


def run():
    field, known = make_lifecycle_corridor(START, GOAL, N_THERMALS, seed=SEED,
                                           born_fraction=BORN_FRACTION, wind=WIND)
    rng = np.random.default_rng(SEED + 1)
    uploaded = [(x + float(rng.normal(0, PRIOR_POS_SIGMA)),
                 y + float(rng.normal(0, PRIOR_POS_SIGMA)), w) for (x, y, w) in known]
    belief = BeliefMap(build_prior(uploaded, (0.0, 0.0), 0.0))
    live_map = ThermalMap(merge_dist=80.0)

    state = GliderState(START[0], START[1], START_H, START_HEADING, AIRSPEED)
    glider = SimpleGlider(state)
    suite = SimulatedSensorSuite(_clean_cfg(), seed=SEED)
    estimator = ThermalEstimator()
    estimator.set_wind(WIND)          # track drifting thermals in the wind frame
    wind_obj = Wind(WIND[0], WIND[1], 0.0, 0.0)
    sm = GuidanceStateMachine(cloud_base=CLOUD_BASE)
    l1 = L1Guidance()
    circling = CirclingControl(l1)
    contact = ContactDetector(lift_threshold=0.8)
    segs, search_total = _search_schedule(AIRSPEED)

    bank = 0.0
    estimate = None
    target = None
    searching = False
    search_t = 0.0
    prev_mode = None
    thermal_entry_h = None
    last_climb_pos = None
    skipping_weak = False
    events = {"confirm": 0, "dead_arrival": 0, "opportunistic": 0, "skipped_weak": 0}
    log = {k: [] for k in ("t", "x", "y", "h", "mode", "phase")}

    n_steps = int(SIM_TIME / DT)
    for k in range(n_steps):
        t = k * DT
        lift = field.vertical_velocity(state.x, state.y, t)
        sink = glider.sink_rate()
        h_dot = glider.step(bank, lift, DT)

        truth = ground_truth_from_sim(t, state, h_dot, wind=WIND)
        snap = suite.read(truth)
        vario = snap.baro.vertical_speed
        estimator.add_measurement(state.x, state.y, vario, sink)
        if k % ESTIMATE_EVERY == 0:
            estimate = estimator.estimate()
        belief.drift(wind_obj, DT)        # candidates track the drifting thermals
        belief.decay(DT)

        # (4) value gate: hide a not-worth-it thermal from the state machine so
        # we don't commit to circling it.
        est_sm = estimate
        if estimate is not None and not worth_climbing(
                estimate, state.h, state.V, low_alt=LOW_ALT, min_climb_comfortable=MIN_CLIMB):
            est_sm = None
            if not skipping_weak:
                events["skipped_weak"] += 1     # count distinct weak thermals, not steps
                skipping_weak = True
        else:
            skipping_weak = False

        mode = sm.update(est_sm, altitude=state.h, position=(state.x, state.y))
        active = sm.active_estimate

        if mode == GuidanceMode.THERMAL and prev_mode != GuidanceMode.THERMAL:
            thermal_entry_h = state.h
        if prev_mode == GuidanceMode.THERMAL and mode == GuidanceMode.CRUISE:
            climbed = thermal_entry_h is not None and (state.h - thermal_entry_h) > 50.0
            if climbed and last_climb_pos is not None:
                live_map.add_or_update(last_climb_pos[0], last_climb_pos[1], 3.0, 50.0, t)  # (3) online
                if target is not None:
                    belief.confirm(target, last_climb_pos[0], last_climb_pos[1], 3.0)
                else:
                    events["opportunistic"] += 1                                            # (3)
            elif target is not None:
                belief.disconfirm(target)
            target = None
            searching = False
            contact.reset()
        prev_mode = mode

        phase = mode.value
        if mode == GuidanceMode.THERMAL and active is not None:
            bank = circling.command(state, active)
            last_climb_pos = (active.x_c, active.y_c)
            searching = False
        elif searching:
            search_t += DT
            if search_t > search_total:
                # (2) dead-arrival: predicted thermal isn't here -> drop it, move on
                if target is not None:
                    belief.disconfirm(target)
                    live_map.mark_dead(target.x, target.y)
                    events["dead_arrival"] += 1
                target = None
                searching = False
                bank = l1.bank_to_point(state, GOAL[0], GOAL[1])
                phase = "search_done"
            else:
                bank = _expanding_figure8(search_t, segs)
                phase = "search"
        else:
            if target is None:
                target = belief.best_target(state.x, state.y, state.h, GOAL)
            if target is None:
                bank = l1.bank_to_point(state, GOAL[0], GOAL[1])
                phase = "to_goal"
            elif math.hypot(target.x - state.x, target.y - state.y) <= SEARCH_ENTER_RADIUS:
                searching = True
                search_t = 0.0
                bank = _expanding_figure8(0.0, segs)
                phase = "search"
            else:
                bank = l1.bank_to_point(state, target.x, target.y)
                phase = "goto"

        for key, val in (("t", t), ("x", state.x), ("y", state.y), ("h", state.h),
                         ("mode", mode.value), ("phase", phase)):
            log[key].append(val)
        if math.hypot(state.x - GOAL[0], state.y - GOAL[1]) <= GOAL_RADIUS:
            break
        if state.h < 0:
            break

    events["confirm"] = sum(1 for c in belief.candidates if c.confirmed)
    return field, belief, live_map, log, events


def summarize(field, belief, live_map, log, events):
    reached = math.hypot(log["x"][-1] - GOAL[0], log["y"][-1] - GOAL[1]) <= GOAL_RADIUS
    print(f"thermals (lifecycle): {len(field.thermals)}, on uploaded map at takeoff: "
          f"{len([t for t in field.thermals if t.birth <= 0])}")
    print(f"confirmed (used): {events['confirm']}   opportunistic (new, off-map): {events['opportunistic']}")
    print(f"dead-arrivals (predicted but gone): {events['dead_arrival']}   weak skipped: {events['skipped_weak']}")
    print(f"altitude: {log['h'][0]:.0f} -> {log['h'][-1]:.0f} m (peak {max(log['h']):.0f}, min {min(log['h']):.0f})")
    print(f"goal: {'REACHED' if reached else 'did not reach'} (final "
          f"{math.hypot(log['x'][-1]-GOAL[0], log['y'][-1]-GOAL[1]):.0f} m away)")


def render(field, log, filename="lifecycle_2d.gif", stride=45, fps=24):
    x = np.array(log["x"]); y = np.array(log["y"]); h = np.array(log["h"])
    t = np.array(log["t"]); mode = log["mode"]
    idx = list(range(0, len(x), stride))
    if idx[-1] != len(x) - 1:
        idx.append(len(x) - 1)
    mc = {"cruise": "#888780", "probe": "#BA7517", "thermal": "#0F6E56"}

    fig, (axp, axh) = plt.subplots(1, 2, figsize=(14, 6.5))
    axp.scatter([START[0]], [START[1]], color="#1D9E75", s=45, zorder=7)
    axp.scatter([GOAL[0]], [GOAL[1]], color="#534AB7", marker="D", s=55, zorder=7)
    axp.set_xlim(min(x.min(), 0) - 80, max(x.max(), GOAL[0]) + 80)
    axp.set_ylim(min(y.min(), 0) - 80, max(y.max(), GOAL[1]) + 80)
    axp.set_aspect("equal"); axp.grid(alpha=0.25)
    axp.set_xlabel("x (m)"); axp.set_ylabel("y (m)")
    axp.set_title("thermals pulse in/out (size = current strength)")
    th_scat = axp.scatter([], [], c=[], s=[], marker="*", zorder=5,
                          cmap="autumn_r", vmin=0, vmax=4.5)
    trail, = axp.plot([], [], color="#185FA5", lw=1.0)
    pdot, = axp.plot([], [], marker="o", ms=8, color="#185FA5")

    axh.plot(t, h, color="#185FA5", lw=1.0)
    axh.axhline(CLOUD_BASE, ls="--", color="#D85A30", lw=1)
    axh.axhline(LOW_ALT, ls=":", color="#A32D2D", lw=1)
    axh.set_xlabel("time (s)"); axh.set_ylabel("altitude (m)"); axh.grid(alpha=0.25)
    axh.set_title("altitude (cloud base --, survival floor :)")
    hdot, = axh.plot([], [], marker="o", ms=8, color="#185FA5")
    txt = axh.text(0.03, 0.95, "", transform=axh.transAxes, va="top", fontsize=11)

    def upd(fi):
        i = idx[fi]
        ti = t[i]
        pts, cols, sizes = [], [], []
        for th in field.thermals:
            w = th.strength(ti)
            if w > 0.05:
                cx, cy = th.center(ti)
                pts.append((cx, cy)); cols.append(w); sizes.append(40 + 80 * w / 4.5)
        if pts:
            th_scat.set_offsets(pts); th_scat.set_array(np.array(cols)); th_scat.set_sizes(sizes)
        else:
            th_scat.set_offsets(np.empty((0, 2)))
        trail.set_data(x[: i + 1], y[: i + 1])
        pdot.set_data([x[i]], [y[i]]); pdot.set_color(mc.get(mode[i], "#185FA5"))
        hdot.set_data([t[i]], [h[i]]); hdot.set_color(mc.get(mode[i], "#185FA5"))
        txt.set_text(f"t={ti:.0f}s  h={h[i]:.0f}m\nmode: {mode[i]}")
        return th_scat, trail, pdot, hdot, txt

    fig.suptitle("Soaring a changing world: lifecycle + belief decay + online map + value decision", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    anim = FuncAnimation(fig, upd, frames=len(idx), blit=False)
    os.makedirs(_OUTPUT_DIR, exist_ok=True)
    out = os.path.abspath(os.path.join(_OUTPUT_DIR, filename))
    anim.save(out, writer=PillowWriter(fps=fps))
    plt.close(fig)
    return out


if __name__ == "__main__":
    field, belief, live_map, log, events = run()
    summarize(field, belief, live_map, log, events)
    print("Saved", render(field, log))
