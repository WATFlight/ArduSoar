"""Explore-then-return mission on the merging random field.

Pre-flight we upload a thermal map (drawn in BLUE). The glider works a thermal to
gain altitude, then pushes OUT to the next uploaded point, surveying every point
in turn (confirm = lift there, or strike it off = empty). The blue points are NOT
constant: as the flight goes on the world changes and each point is updated as it
is explored. Only once all the points are explored does the glider head home.

Realism: the thermals DRIFT downwind. We do not hand the wind to the glider — it
estimates the wind itself from its sensors (GPS ground velocity vs pitot airspeed
along the compass heading), then (a) fits/tracks the moving cores in that wind
frame and (b) advects the uploaded map downwind so the survey targets keep up.

    python explore.py
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
from glider_model.motor import ElectricSustainer
from thermal_model.random_field import make_random_world, make_uploaded_map
from thermal_model.lifecycle_thermal import LifecycleThermal
from thermal_model.merging_field import MergingField
from thermal_estimator.estimator import ThermalEstimator
from controller.state_machine import GuidanceStateMachine, GuidanceMode
from controller.l1_guidance import L1Guidance
from controller.circling_control import CirclingControl
from sensors.simulated import SimulatedSensorSuite, ground_truth_from_sim, SensorConfig, NoiseSpec
from estimation.state_fusion import PassthroughFusion
from estimation.wind_estimator import SimpleWindEstimator
from navigation.thermal_prior import build_prior, BeliefMap
from navigation.decision import worth_climbing
from lifecycle_search import _search_schedule, _expanding_figure8

ORIGIN = (0.0, 0.0)
BOUNDS = (-2000.0, 2000.0, -2000.0, 2000.0)
START_H = 740.0
CLOUD_BASE = 800.0
LOW_ALT = 320.0
MIN_CLIMB = 0.3
LAND_RADIUS = 120.0
RETURN_ALT = 300.0              # below this with nothing to climb -> abort survey, go home
MAX_TIME = 6000.0
INITIAL_COUNT = 12
SPAWN_RATE = 0.012
MERGE_DIST = 75.0
SEARCH_ENTER_RADIUS = 45.0
WIND = (0.9, -0.55)              # light drift (~1.1 m/s); the glider estimates it
# --- "fuel" monitor: altitude -> glide range, PLUS a real electric battery ---
GLIDE_RATIO = 22.0
HOME_RESERVE = 130.0           # altitude (m) kept in hand to arrive + land at home
ENERGY_MARGIN = 500.0          # spare range (m) below which we play it safe
MOTOR_FLOOR = 260.0            # sink below this with no lift -> run the sustainer
MOTOR_CEIL = 430.0            # climb back to here under motor, then shut it off
MAP_DECAY_TAU = 4000.0         # uploaded map ages slowly -> glider commits to far points
                               # (sinks out between them; the sustainer is what saves it)
SEED = 4
_OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "output")


def _clean_cfg():
    cfg = SensorConfig()
    for f in ("accel", "gyro", "gps_pos", "gps_vel", "compass", "pitot",
              "baro_alt", "baro_vspeed", "temp", "humidity"):
        setattr(cfg, f, NoiseSpec(std=0.0))
    return cfg


def _build_field():
    base = make_random_world(BOUNDS, MAX_TIME, wind=WIND, seed=SEED,
                             initial_count=INITIAL_COUNT, spawn_rate=SPAWN_RATE,
                             grow_range=(90, 180), hold_range=(900, 1600), decay_range=(250, 450))
    rng = np.random.default_rng(SEED + 2)
    twins = []
    for th in rng.choice(base.thermals, size=min(3, len(base.thermals)), replace=False):
        twins.append(LifecycleThermal(
            th.x + float(rng.uniform(-45, 45)), th.y + float(rng.uniform(-45, 45)),
            float(rng.uniform(2.8, 4.0)), float(rng.uniform(45, 60)),
            th.birth + float(rng.uniform(-60, 60)), th.t_grow, th.t_hold, th.t_decay,
            wind=WIND))
    base.thermals += twins
    return MergingField(base.thermals, merge_dist=MERGE_DIST)


def _pending(cands, explored_t, abandoned):
    """Points still worth surveying: not yet surveyed and not written off."""
    return [i for i in range(len(cands)) if explored_t[i] is None and not abandoned[i]]


def _glide_range(h, extra_climb=0.0):
    """How far the glider can still travel (m): glide from the current height,
    plus any metres the motor battery can still climb (each climbed metre buys
    GLIDE_RATIO metres of glide). ``extra_climb`` = battery's available climb."""
    return max(0.0, h - HOME_RESERVE + extra_climb) * GLIDE_RATIO


def _home_margin(state, extra_climb=0.0):
    """The fuel gauge: spare range after deducting the trip home (m). >0 means it
    can still reach home (gliding, plus motoring if there is charge)."""
    return _glide_range(state.h, extra_climb) - math.hypot(state.x - ORIGIN[0], state.y - ORIGIN[1])


def _select_target(state, cands, pending, extra_climb=0.0):
    """Pick the next point to work, with an eye on getting home.

    A point is 'home-safe' if we can fly out to it AND still get home from there
    on the energy we have now — altitude plus whatever the battery can still climb
    (worst case: that thermal turns out dead). When the gauge is healthy we take
    the nearest pending point; when it runs low (height + battery) we prefer points
    CLOSER TO HOME so working them does not strand us."""
    if not pending:
        return None
    reach = _glide_range(state.h, extra_climb)

    def out_and_back(i):
        return (math.hypot(cands[i].x - state.x, cands[i].y - state.y)
                + math.hypot(cands[i].x - ORIGIN[0], cands[i].y - ORIGIN[1]))

    safe = [i for i in pending if out_and_back(i) <= reach]   # reach it and still get home
    pool = safe or pending
    if _home_margin(state, extra_climb) < ENERGY_MARGIN:      # low fuel -> stay near home
        return min(pool, key=lambda i: math.hypot(cands[i].x - ORIGIN[0], cands[i].y - ORIGIN[1]))
    return min(pool, key=lambda i: math.hypot(cands[i].x - state.x, cands[i].y - state.y))


def run():
    field = _build_field()
    uploaded = make_uploaded_map(field, upload_time=0.0, seed=SEED + 1,
                                 offset=(40.0, -25.0), rotation_deg=1.0, pos_noise=22.0)
    belief = BeliefMap(build_prior(uploaded, (0.0, 0.0), 0.0))
    cands = belief.candidates
    cand0 = [(c.x, c.y) for c in cands]  # uploaded positions, before any drift
    explored_t = [None] * len(cands)     # time each blue point was surveyed
    explored_ok = [False] * len(cands)   # True if lift found there
    abandoned = [False] * len(cands)     # written off (map says it's probably gone)

    state = GliderState(ORIGIN[0], ORIGIN[1], START_H, math.radians(20), AIRSPEED)
    glider = SimpleGlider(state)
    suite = SimulatedSensorSuite(_clean_cfg(), seed=SEED)
    estimator = ThermalEstimator()
    sustainer = ElectricSustainer()              # electric motor + depletable battery
    fusion = PassthroughFusion()
    wind_est = SimpleWindEstimator(alpha=0.05)   # estimate wind from the sensors
    sm = GuidanceStateMachine(cloud_base=CLOUD_BASE)
    l1 = L1Guidance()
    circling = CirclingControl(l1)
    segs, search_total = _search_schedule(AIRSPEED)

    bank = 0.0
    estimate = None
    target_i = None
    searching = False
    search_t = 0.0
    prev_mode = None
    thermal_entry_h = None
    homebound = False
    log = {k: [] for k in ("t", "x", "y", "h", "mode", "margin", "soc", "motor")}
    snaps = []   # (t, [(x,y,state) per candidate]) sampled for animation

    for k in range(int(MAX_TIME / DT)):
        t = k * DT
        # --- electric sustainer: kick in when low with no lift, climb back, shut off ---
        in_thermal = prev_mode == GuidanceMode.THERMAL
        if sustainer.on:
            want_motor = state.h < MOTOR_CEIL and not in_thermal
        else:
            want_motor = state.h < MOTOR_FLOOR and not in_thermal
        motor_climb = sustainer.step(want_motor, DT)

        lift = field.vertical_velocity(state.x, state.y, t)
        sink = glider.sink_rate()
        h_dot = glider.step(bank, lift + motor_climb, DT)
        if state.h < 0.0:
            break

        # the vario senses AIR motion only — strip the motor's contribution so the
        # estimator never mistakes engine thrust for a thermal.
        snap = suite.read(ground_truth_from_sim(t, state, h_dot - motor_climb, wind=WIND))
        vehicle = fusion.update(snap)
        wind = wind_est.update(vehicle)              # wind from GPS + pitot + compass
        estimator.set_wind(wind)                     # track the drifting cores
        estimator.add_measurement(state.x, state.y, snap.baro.vertical_speed, sink)
        if k % ESTIMATE_EVERY == 0:
            estimate = estimator.estimate()
        belief.decay(DT, tau=MAP_DECAY_TAU)          # the uploaded map ages
        belief.drift(wind, DT)                       # advect the uploaded map downwind

        est_sm = estimate if (estimate is not None and worth_climbing(
            estimate, state.h, state.V, low_alt=LOW_ALT, min_climb_comfortable=MIN_CLIMB)) else None
        mode = sm.update(est_sm, altitude=state.h, position=(state.x, state.y))
        active = sm.active_estimate

        if mode == GuidanceMode.THERMAL and prev_mode != GuidanceMode.THERMAL:
            thermal_entry_h = state.h
        if prev_mode == GuidanceMode.THERMAL and mode == GuidanceMode.CRUISE:
            climbed = thermal_entry_h is not None and (state.h - thermal_entry_h) > 50.0
            if target_i is not None and climbed:               # surveyed: lift confirmed
                explored_t[target_i] = t; explored_ok[target_i] = True
            target_i = None
            searching = False
        prev_mode = mode

        # Write off points the aging map no longer trusts (drifted away / died).
        for j in range(len(cands)):
            if explored_t[j] is None and not abandoned[j] and cands[j].prob < belief.min_prob:
                abandoned[j] = True
        if target_i is not None and abandoned[target_i]:
            target_i = None; searching = False
        pending = _pending(cands, explored_t, abandoned)

        if mode == GuidanceMode.THERMAL and active is not None:
            bank = circling.command(state, active)
            searching = False
        elif searching:
            search_t += DT
            if search_t > search_total:                        # surveyed: empty
                if target_i is not None:
                    explored_t[target_i] = t; explored_ok[target_i] = False
                target_i = None; searching = False
                bank = l1.bank_to_point(state, ORIGIN[0], ORIGIN[1])
            else:
                bank = _expanding_figure8(search_t, segs)
        else:
            # fuel check before committing to / leaving a thermal: with the energy we
            # have (height + whatever the battery can still climb), can we get home?
            avail = sustainer.available_climb_m()
            battery_dead = sustainer.charge_wh <= 0.0
            have_fuel = (_home_margin(state, avail) > 0.0
                         and not (battery_dead and state.h < RETURN_ALT))
            if pending and have_fuel:                          # energy + points to survey
                if target_i is None:                           # energy-aware pick (near home if low)
                    target_i = _select_target(state, cands, pending, avail)
                tx, ty = cands[target_i].x, cands[target_i].y
                if math.hypot(tx - state.x, ty - state.y) <= SEARCH_ENTER_RADIUS:
                    searching = True; search_t = 0.0
                    bank = _expanding_figure8(0.0, segs)
                else:
                    bank = l1.bank_to_point(state, tx, ty)     # push OUT to the next point
            else:
                # survey done, or not enough energy to keep working -> head home
                homebound = not pending
                target_i = None
                d_home = math.hypot(state.x - ORIGIN[0], state.y - ORIGIN[1])
                if d_home <= LAND_RADIUS:
                    break                                      # made it home
                bank = l1.bank_to_point(state, ORIGIN[0], ORIGIN[1])

        for key, val in (("t", t), ("x", state.x), ("y", state.y), ("h", state.h),
                         ("mode", mode.value),
                         ("margin", _home_margin(state, sustainer.available_climb_m())),
                         ("soc", sustainer.soc()), ("motor", 1.0 if sustainer.on else 0.0)):
            log[key].append(val)

    n_done = sum(1 for et in explored_t if et is not None)
    home_dist = math.hypot(log["x"][-1] - ORIGIN[0], log["y"][-1] - ORIGIN[1])
    motor_time = sum(log["motor"]) * DT
    info = {"surveyed": n_done, "total": len(cands), "with_lift": sum(explored_ok),
            "abandoned": sum(abandoned), "flight_time": log["t"][-1], "home_dist": home_dist,
            "homebound": homebound, "explored_t": explored_t, "explored_ok": explored_ok,
            "abandoned_flags": abandoned, "cands": cand0,
            "soc_end": log["soc"][-1], "motor_time": motor_time}
    return field, info, log


def summarize(info):
    print(f"surveyed {info['surveyed']}/{info['total']} uploaded points "
          f"({info['with_lift']} had lift), abandoned {info['abandoned']} stale ones, "
          f"in {info['flight_time']/60:.1f} min")
    print(f"{'headed home' if info['homebound'] else 'did NOT finish survey'}; "
          f"ended {info['home_dist']:.0f} m from home")
    print(f"electric sustainer ran {info['motor_time']:.0f} s; battery {info['soc_end']*100:.0f}% left")


def render(field, info, log, filename="explore_2d.gif", stride=80, fps=24):
    x = np.array(log["x"]); y = np.array(log["y"]); h = np.array(log["h"]); t = np.array(log["t"])
    marg = np.array(log["margin"]); marg0 = max(marg[0], 1.0)   # fuel gauge baseline
    soc = np.array(log["soc"]); motor = np.array(log["motor"])
    mode = log["mode"]
    idx = list(range(0, len(x), stride)) + [len(x) - 1]
    mc = {"cruise": "#888780", "probe": "#BA7517", "thermal": "#0F6E56"}
    cands = info["cands"]; et = info["explored_t"]; ok = info["explored_ok"]
    aband = info["abandoned_flags"]

    fig, (axp, axh) = plt.subplots(1, 2, figsize=(13, 6.2))
    axp.scatter([ORIGIN[0]], [ORIGIN[1]], color="#534AB7", marker="s", s=70, zorder=8, label="home")
    axp.set_xlim(BOUNDS[0] - 60, BOUNDS[1] + 60); axp.set_ylim(BOUNDS[2] - 60, BOUNDS[3] + 60)
    axp.set_aspect("equal"); axp.grid(alpha=0.25); axp.legend(fontsize=9, loc="upper left")
    axp.set_xlabel("x (m)"); axp.set_ylabel("y (m)")
    axp.set_title("blue = uploaded points (unsurveyed); green = lift, grey = empty")
    th_scat = axp.scatter([], [], c=[], s=[], marker="*", zorder=5, cmap="autumn_r", vmin=0, vmax=5)
    cand_scat = axp.scatter([cx for cx, cy in cands], [cy for cx, cy in cands],
                            c=["#185FA5"] * len(cands), s=70, marker="o",
                            edgecolors="white", linewidths=0.6, zorder=6)
    cand0 = np.array(cands) if cands else np.empty((0, 2))   # uploaded positions
    trail, = axp.plot([], [], color="#185FA5", lw=0.9)
    dot, = axp.plot([], [], marker="o", ms=8, color="#185FA5")

    axh.plot(t, h, color="#185FA5", lw=1.0)
    axh.axhline(CLOUD_BASE, ls="--", color="#D85A30", lw=1)
    axh.axhline(0, ls="-", color="#A32D2D", lw=1)
    axh.fill_between(t, 0, h.max() * 1.05, where=motor > 0.5, color="#C75D2C",
                     alpha=0.12, step="pre", label="motor on")
    axh.set_xlabel("time (s)"); axh.set_ylabel("altitude (m)"); axh.grid(alpha=0.25)
    axh.set_title("altitude (shaded = electric sustainer running)")
    hdot, = axh.plot([], [], marker="o", ms=8, color="#185FA5")
    txt = axh.text(0.03, 0.95, "", transform=axh.transAxes, va="top", fontsize=11)

    def upd(fi):
        i = idx[fi]; ti = t[i]
        eff = field.effective(ti)
        pts = [(cx, cy) for cx, cy, W, R, n in eff if W > 0.1]
        cols = [W for cx, cy, W, R, n in eff if W > 0.1]
        sizes = [40 + 35 * W + 55 * (n - 1) for cx, cy, W, R, n in eff if W > 0.1]
        th_scat.set_offsets(pts if pts else np.empty((0, 2)))
        if pts:
            th_scat.set_array(np.array(cols)); th_scat.set_sizes(sizes)
        if len(cand0):                                  # advect the map downwind
            drifted = cand0 + np.array([WIND[0] * ti, WIND[1] * ti])
            cand_scat.set_offsets(drifted)
        cand_colors = []
        for j in range(len(cands)):
            if et[j] is not None and ti >= et[j]:
                cand_colors.append("#1D9E75" if ok[j] else "#B4B2A9")  # green=lift / grey=empty
            elif aband[j]:
                cand_colors.append("#D9B36B")           # tan: written off (map went stale)
            else:
                cand_colors.append("#185FA5")           # blue: not yet surveyed
        cand_scat.set_color(cand_colors)
        trail.set_data(x[: i + 1], y[: i + 1])
        dot.set_data([x[i]], [y[i]]); dot.set_color(mc.get(mode[i], "#185FA5"))
        hdot.set_data([t[i]], [h[i]]); hdot.set_color(mc.get(mode[i], "#185FA5"))
        pct = max(0.0, min(1.0, marg[i] / marg0)) * 100.0    # fuel gauge: range-home margin
        gauge = "LOW" if marg[i] < ENERGY_MARGIN else "ok"
        mtr = "  [MOTOR]" if motor[i] > 0.5 else ""
        txt.set_text(f"t={ti:.0f}s ({ti/60:.1f} min)  h={h[i]:.0f}m\nmode: {mode[i]}{mtr}"
                     f"\nhome-reach fuel: {marg[i]:.0f} m ({pct:.0f}%, {gauge})"
                     f"\nbattery: {soc[i]*100:.0f}%")
        return th_scat, cand_scat, trail, dot, hdot, txt

    spd = math.hypot(*WIND)
    axp.annotate("", xy=(BOUNDS[1] - 250 + WIND[0] * 90, BOUNDS[3] - 250 + WIND[1] * 90),
                 xytext=(BOUNDS[1] - 250, BOUNDS[3] - 250),
                 arrowprops=dict(arrowstyle="-|>", color="#444441", lw=1.4))
    axp.text(BOUNDS[1] - 250, BOUNDS[3] - 320, f"wind {spd:.1f} m/s", fontsize=8, color="#444441")
    fig.suptitle("Survey the uploaded map (drifting downwind, wind self-estimated), then return home",
                 fontsize=12)
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
