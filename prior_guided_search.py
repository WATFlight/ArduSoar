"""Prior-guided thermal search (the "upload a thermal map + wind" strategy).

Before flight: upload candidate thermal *source* locations and the wind. Thermals
drift downwind, so the predicted current position of each candidate = source +
wind drift. In flight the glider:

  1. flies to the highest-probability *reachable* candidate (toward the goal),
  2. searches locally there and watches its variometer for thermal contact,
  3. on contact, captures + circles + climbs (reusing the capture state machine),
  4. CONFIRMS that candidate (refines its position) and hops to the next,
  5. if it searches a candidate and finds nothing, DISCONFIRMS it and moves on.

All measurements flow through the simulated sensor suite, so this runs on the
same interfaces the real GPS/IMU/baro will fill later.

    python prior_guided_search.py
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
from thermal_model.thermal import GaussianThermal
from thermal_model.thermal_field import ThermalField
from thermal_estimator.estimator import ThermalEstimator
from controller.state_machine import GuidanceStateMachine, GuidanceMode
from controller.l1_guidance import L1Guidance
from controller.circling_control import CirclingControl
from controller.probe_control import ProbeControl
from sensors.simulated import SimulatedSensorSuite, ground_truth_from_sim, SensorConfig, NoiseSpec
from navigation.thermal_prior import build_prior, BeliefMap, CandidatePoint
from navigation.contact_detector import ContactDetector

# --- scenario ---
START = (0.0, 0.0)
GOAL = (4200.0, 3000.0)
START_H = 350.0
CLOUD_BASE = 700.0
SIM_TIME = 2200.0
WIND = (3.0, 1.5)            # m/s, blows toward +x/+y
DRIFT_DIST = 160.0          # how far thermals drift downwind from their source
PRIOR_POS_SIGMA = 50.0      # uploaded-map position error
SEARCH_ENTER_RADIUS = 45.0  # arrive within this of a candidate, then latch into search
# Expanding figure-8 search: 3 loops, tight -> wide (bank decreasing => radius
# increasing). After the last loop (the boundary) with no contact, give up and
# let the map pick the next point instead of burning more energy circling.
SEARCH_BANKS_DEG = (35.0, 28.0, 22.0)
CLIMB_TO_CONFIRM = 50.0     # only confirm a candidate if we actually climbed here
GOAL_RADIUS = 90.0
SEED = 5


def _clean_sensor_config() -> SensorConfig:
    """Noise-free sensors for this demo: the prior-guided *logic* is what we are
    validating. Realistic sensor noise fools the 1/(1+mse) confidence metric into
    false captures (a separate, known issue -> the chi-squared-confidence upgrade)."""
    cfg = SensorConfig()
    for f in ("accel", "gyro", "gps_pos", "gps_vel", "compass", "pitot",
              "baro_alt", "baro_vspeed", "temp", "humidity"):
        setattr(cfg, f, NoiseSpec(std=0.0))
    return cfg

_OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "output")


def _build_world():
    rng = np.random.default_rng(SEED)
    wx, wy = WIND
    sp = math.hypot(wx, wy)
    ux, uy = wx / sp, wy / sp

    # thermal SOURCES roughly along the route to the goal
    n = 6
    sources = []
    for i in range(n):
        f = (i + 1) / (n + 1)
        sx = START[0] + f * (GOAL[0] - START[0]) + float(rng.normal(0, 150))
        sy = START[1] + f * (GOAL[1] - START[1]) + float(rng.normal(0, 150))
        sources.append((sx, sy, float(rng.uniform(3.0, 4.2))))

    # TRUE thermals = sources drifted downwind
    true_thermals = [
        GaussianThermal(sx + ux * DRIFT_DIST, sy + uy * DRIFT_DIST, w, float(rng.uniform(50, 65)))
        for (sx, sy, w) in sources
    ]
    field = ThermalField(true_thermals)

    # UPLOADED map: source positions + error, drop one (missing), add two spurious
    uploaded = []
    for i, (sx, sy, w) in enumerate(sources):
        if i == 2:
            continue  # this real thermal is missing from the upload
        uploaded.append((sx + float(rng.normal(0, PRIOR_POS_SIGMA)),
                         sy + float(rng.normal(0, PRIOR_POS_SIGMA)),
                         w * float(rng.uniform(0.8, 1.1))))
    for _ in range(2):  # spurious points with no real thermal under them
        uploaded.append((float(rng.uniform(800, 3600)), float(rng.uniform(400, 2600)),
                         float(rng.uniform(2.6, 3.6))))

    # candidates = uploaded sources shifted by the (known) wind drift
    candidates = build_prior(uploaded, WIND, DRIFT_DIST)
    return field, true_thermals, BeliefMap(candidates)


def _expanding_search_schedule(V, banks_deg=SEARCH_BANKS_DEG):
    """Pre-compute the expanding figure-8: one (start, half-period, bank) per
    loop, plus the total search time (the boundary)."""
    segs = []
    cum = 0.0
    for d in banks_deg:
        phi = math.radians(d)
        t_loop = 2.0 * math.pi / (9.81 * math.tan(phi) / V)   # one full circle
        segs.append((cum, t_loop, phi))                        # a figure-8 = 2*t_loop
        cum += 2.0 * t_loop
    return segs, cum


def _expanding_figure8(te, segs):
    """Bank command for the expanding search at elapsed search time ``te``.
    Each loop: right lobe (−phi) then left lobe (+phi), with phi shrinking
    (radius growing) loop by loop."""
    for start, t_loop, phi in segs:
        if te < start + 2.0 * t_loop:
            return -phi if (te - start) < t_loop else phi
    return 0.0  # past the boundary


def run():
    field, true_thermals, belief = _build_world()
    state = GliderState(START[0], START[1], START_H, START_HEADING, AIRSPEED)
    glider = SimpleGlider(state)
    suite = SimulatedSensorSuite(_clean_sensor_config(), seed=SEED)
    estimator = ThermalEstimator()
    sm = GuidanceStateMachine(cloud_base=CLOUD_BASE)
    l1 = L1Guidance()
    circling = CirclingControl(l1)
    probe = ProbeControl()
    contact = ContactDetector(lift_threshold=0.9)
    search_segs, search_total = _expanding_search_schedule(AIRSPEED)

    bank = 0.0
    estimate = None
    target = None
    search_t = 0.0
    prev_mode = None
    last_climb_pos = None
    thermal_entry_h = None
    searching = False        # latched once we commit to searching a candidate
    log = {k: [] for k in ("t", "x", "y", "h", "mode", "phase", "contact")}

    n_steps = int(SIM_TIME / DT)
    for k in range(n_steps):
        t = k * DT
        lift = field.vertical_velocity(state.x, state.y)
        sink = glider.sink_rate()
        h_dot = glider.step(bank, lift, DT)

        # everything through the (simulated) sensors
        truth = ground_truth_from_sim(t, state, h_dot, wind=WIND)
        snap = suite.read(truth)
        vario = snap.baro.vertical_speed
        in_contact, _ = contact.update(vario + sink)

        estimator.add_measurement(state.x, state.y, vario, sink)
        if k % ESTIMATE_EVERY == 0:
            estimate = estimator.estimate()

        mode = sm.update(estimate, altitude=state.h, position=(state.x, state.y))
        active = sm.active_estimate

        if mode == GuidanceMode.THERMAL and prev_mode != GuidanceMode.THERMAL:
            thermal_entry_h = state.h
        # left a thermal -> confirm only if we actually climbed here, else reject
        if prev_mode == GuidanceMode.THERMAL and mode == GuidanceMode.CRUISE:
            climbed = thermal_entry_h is not None and (state.h - thermal_entry_h) > CLIMB_TO_CONFIRM
            if target is not None:
                if climbed and last_climb_pos is not None:
                    belief.confirm(target, last_climb_pos[0], last_climb_pos[1], 3.5)
                else:
                    belief.disconfirm(target)
            target = None
            search_t = 0.0
            searching = False
            contact.reset()
        prev_mode = mode

        phase = mode.value
        if mode == GuidanceMode.THERMAL and active is not None:
            bank = circling.command(state, active)
            last_climb_pos = (active.x_c, active.y_c)
            searching = False
        elif mode == GuidanceMode.PROBE and active is not None:
            bank = probe.command(state, active)
        elif searching:
            # Latched search: run the bounded expanding figure-8 to completion,
            # WITHOUT re-deciding every step (that caused the bounce-in-place).
            search_t += DT
            if search_t > search_total:
                # 3 expanding loops, no thermal -> give up, let the map pick the
                # next point and glide STRAIGHT there (no more circling = saves energy).
                belief.disconfirm(target)
                target = None
                searching = False
                bank = l1.bank_to_point(state, GOAL[0], GOAL[1])
                phase = "search_done"
            else:
                bank = _expanding_figure8(search_t, search_segs)
                phase = "search"
        else:
            # CRUISE: fly straight to the best map candidate; latch into search on arrival.
            if target is None:
                target = belief.best_target(state.x, state.y, state.h, GOAL)
            if target is None:
                bank = l1.bank_to_point(state, GOAL[0], GOAL[1])
                phase = "to_goal"
            elif math.hypot(target.x - state.x, target.y - state.y) <= SEARCH_ENTER_RADIUS:
                searching = True
                search_t = 0.0
                bank = _expanding_figure8(0.0, search_segs)
                phase = "search"
            else:
                bank = l1.bank_to_point(state, target.x, target.y)
                phase = "goto"

        for key, val in (("t", t), ("x", state.x), ("y", state.y), ("h", state.h),
                         ("mode", mode.value), ("phase", phase), ("contact", in_contact)):
            log[key].append(val)

        if math.hypot(state.x - GOAL[0], state.y - GOAL[1]) <= GOAL_RADIUS:
            break

    return field, true_thermals, belief, log


def summarize(field, belief, log):
    confirmed = [c for c in belief.candidates if c.confirmed]
    disconfirmed = [c for c in belief.candidates if c.visited and not c.confirmed]
    reached = math.hypot(log["x"][-1] - GOAL[0], log["y"][-1] - GOAL[1]) <= GOAL_RADIUS
    print(f"candidates: {len(belief.candidates)} (uploaded prior)")
    print(f"confirmed (found lift): {len(confirmed)}")
    print(f"disconfirmed (searched, empty): {len(disconfirmed)}")
    print(f"altitude: {log['h'][0]:.0f} -> {log['h'][-1]:.0f} m  (peak {max(log['h']):.0f})")
    print(f"goal: {'REACHED' if reached else 'did not reach'} (final {math.hypot(log['x'][-1]-GOAL[0], log['y'][-1]-GOAL[1]):.0f} m away)")


def render(field, belief, log, filename="prior_guided_2d.gif", stride=40, fps=24):
    x = np.array(log["x"]); y = np.array(log["y"]); h = np.array(log["h"])
    t = np.array(log["t"]); mode = log["mode"]
    idx = list(range(0, len(x), stride))
    if idx[-1] != len(x) - 1:
        idx.append(len(x) - 1)
    mc = {"cruise": "#888780", "probe": "#BA7517", "thermal": "#0F6E56"}

    fig, (axp, axh) = plt.subplots(1, 2, figsize=(14, 6.5))
    for th in field.thermals:
        axp.scatter([th.x_c], [th.y_c], color="#D85A30", marker="*", s=140, zorder=6)
        axp.add_patch(plt.Circle((th.x_c, th.y_c), th.R_th, color="#D85A30", fill=False, ls="--", alpha=0.35))
    for c in belief.candidates:
        col = "#1D9E75" if c.confirmed else ("#A32D2D" if c.visited else "#888780")
        axp.scatter([c.x], [c.y], color=col, marker="o", s=40, zorder=5,
                    edgecolors="white", linewidths=0.5)
    axp.scatter([START[0]], [START[1]], color="#1D9E75", s=45, zorder=7)
    axp.scatter([GOAL[0]], [GOAL[1]], color="#534AB7", marker="D", s=55, zorder=7)
    axp.set_xlim(x.min() - 80, max(x.max(), GOAL[0]) + 80)
    axp.set_ylim(y.min() - 80, max(y.max(), GOAL[1]) + 80)
    axp.set_aspect("equal"); axp.grid(alpha=0.25)
    axp.set_xlabel("x (m)"); axp.set_ylabel("y (m)")
    axp.set_title("orange*=true thermal  green=confirmed  red=disconfirmed  grey=unvisited")
    trail, = axp.plot([], [], color="#185FA5", lw=1.0)
    pdot, = axp.plot([], [], marker="o", ms=8, color="#185FA5")

    axh.plot(t, h, color="#185FA5", lw=1.0)
    axh.axhline(CLOUD_BASE, ls="--", color="#D85A30", lw=1)
    axh.set_xlabel("time (s)"); axh.set_ylabel("altitude (m)"); axh.grid(alpha=0.25)
    axh.set_title("altitude (thermal hopping)")
    hdot, = axh.plot([], [], marker="o", ms=8, color="#185FA5")
    txt = axh.text(0.03, 0.95, "", transform=axh.transAxes, va="top", fontsize=11)

    def upd(fi):
        i = idx[fi]
        trail.set_data(x[: i + 1], y[: i + 1])
        pdot.set_data([x[i]], [y[i]]); pdot.set_color(mc.get(mode[i], "#185FA5"))
        hdot.set_data([t[i]], [h[i]]); hdot.set_color(mc.get(mode[i], "#185FA5"))
        txt.set_text(f"t={t[i]:.0f}s  h={h[i]:.0f}m\nmode: {mode[i]}")
        return trail, pdot, hdot, txt

    fig.suptitle("Prior-guided search: fly to likely points, validate by flying", fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    anim = FuncAnimation(fig, upd, frames=len(idx), blit=False)
    os.makedirs(_OUTPUT_DIR, exist_ok=True)
    out = os.path.abspath(os.path.join(_OUTPUT_DIR, filename))
    anim.save(out, writer=PillowWriter(fps=fps))
    plt.close(fig)
    return out


if __name__ == "__main__":
    field, true_thermals, belief, log = run()
    summarize(field, belief, log)
    out = render(field, belief, log)
    print(f"Saved animation to {out}")
