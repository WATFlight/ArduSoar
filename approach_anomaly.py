"""Handling a thermal that shifts or vanishes after you've locked onto it.

The glider locks a thermal from its sensors and circles it. Then, mid-climb, the
thermal either:
  * SHIFTS a long way (gust / wandering core), or
  * VANISHES (dies).

Handling: a short moving-average of the lift detects the loss. The glider then
runs a bounded EXPANDING re-acquire search outward from the last known core:
  - shifted  -> the search re-finds it and re-centers (keeps climbing),
  - vanished -> the search comes up empty, so it gives up and flies on.

    python approach_anomaly.py
"""

import math
import os
from collections import deque

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

from config import DT, AIRSPEED
from glider_model.glider import GliderState, SimpleGlider
from thermal_estimator.estimator import ThermalEstimator
from controller.l1_guidance import L1Guidance
from controller.circling_control import CirclingControl

THERMAL_A = (700.0, 500.0)
W0, R = 4.0, 55.0
TRIGGER_T = 80.0                 # anomaly fires while circling
SHIFT = (80.0, 55.0)             # ~97 m jump (large, but within re-acquire reach)
START_H = 560.0
SIM_TIME = 480.0
GOAL = (3200.0, 2300.0)          # where it heads if it gives up
LOST_LIFT = 0.4                  # avg lift below this while circling ...
LOST_HOLD = 5.0                  # ... for this long => declare it lost
REFOUND_LIFT = 1.2               # avg lift above this in re-acquire => re-found
REACQUIRE_BANKS = (30.0, 24.0)   # 2 expanding loops (~82 s, reach ~116 m)
_OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "output")


class AnomalyThermal:
    def __init__(self, case):
        self.case = case

    def center(self, t):
        if self.case == "shift" and t >= TRIGGER_T:
            return (THERMAL_A[0] + SHIFT[0], THERMAL_A[1] + SHIFT[1])
        return THERMAL_A

    def alive(self, t):
        return not (self.case == "vanish" and t >= TRIGGER_T)

    def lift(self, x, y, t):
        if not self.alive(t):
            return 0.0
        cx, cy = self.center(t)
        return W0 * math.exp(-((x - cx) ** 2 + (y - cy) ** 2) / R ** 2)


def _schedule(V, banks=REACQUIRE_BANKS):
    segs, cum = [], 0.0
    for d in banks:
        phi = math.radians(d)
        tl = 2.0 * math.pi / (9.81 * math.tan(phi) / V)
        segs.append((cum, tl, phi))
        cum += 2.0 * tl
    return segs, cum


def _ef8(te, segs):
    for s, tl, phi in segs:
        if te < s + 2.0 * tl:
            return -phi if (te - s) < tl else phi
    return 0.0


def run(case):
    th = AnomalyThermal(case)
    st = GliderState(0.0, 0.0, START_H, math.radians(35), AIRSPEED)
    g = SimpleGlider(st)
    est = ThermalEstimator()
    l1 = L1Guidance()
    circ = CirclingControl(l1)
    segs, reac_total = _schedule(AIRSPEED)

    mode = "approach"
    bank = 0.0
    e = None
    last_est = None
    reac_t = 0.0
    lost_t = 0.0
    liftbuf = deque(maxlen=20)   # ~2 s of lift
    events = []
    log = {k: [] for k in ("t", "x", "y", "h", "mode", "cx", "cy", "alive")}

    for k in range(int(SIM_TIME / DT)):
        t = k * DT
        lift = th.lift(st.x, st.y, t)
        sink = g.sink_rate()
        h_dot = g.step(bank, lift, DT)
        est.add_measurement(st.x, st.y, h_dot, sink)
        if k % 5 == 0:
            e = est.estimate()
        liftbuf.append(h_dot + sink)
        avg = sum(liftbuf) / len(liftbuf)

        if mode == "approach":
            if e is not None and avg > 0.8:
                mode = "circle"; last_est = e
                events.append((t, "locked & circling"))
                bank = circ.command(st, last_est)
            else:
                bank = l1.bank_to_point(st, *THERMAL_A)
        elif mode == "circle":
            lost_t = lost_t + DT if avg < LOST_LIFT else 0.0
            if lost_t > LOST_HOLD:
                mode = "reacquire"; reac_t = 0.0; lost_t = 0.0
                events.append((t, "LOST the thermal"))
                bank = _ef8(0.0, segs)
            else:
                if e is not None:
                    last_est = e
                bank = circ.command(st, last_est)
        elif mode == "reacquire":
            reac_t += DT
            if e is not None and avg > REFOUND_LIFT:
                mode = "circle"; last_est = e
                events.append((t, "RE-ACQUIRED (thermal had shifted)"))
                bank = circ.command(st, last_est)
            elif reac_t > reac_total:
                mode = "gave_up"
                events.append((t, "gave up (thermal vanished) -> fly on"))
                bank = l1.bank_to_point(st, *GOAL)
            else:
                bank = _ef8(reac_t, segs)
        else:  # gave_up
            bank = l1.bank_to_point(st, *GOAL)

        c = th.center(t)
        for key, val in (("t", t), ("x", st.x), ("y", st.y), ("h", st.h),
                         ("mode", mode), ("cx", c[0]), ("cy", c[1]), ("alive", th.alive(t))):
            log[key].append(val)

    return log, events


def render(case, log, filename, stride=20, fps=22):
    x = np.array(log["x"]); y = np.array(log["y"]); h = np.array(log["h"]); t = np.array(log["t"])
    mode = log["mode"]
    idx = list(range(0, len(x), stride)) + [len(x) - 1]
    mc = {"approach": "#888780", "circle": "#0F6E56", "reacquire": "#BA7517", "gave_up": "#A32D2D"}

    fig, (axp, axh) = plt.subplots(1, 2, figsize=(13, 6))
    axp.scatter([0], [0], color="#1D9E75", s=40, zorder=6)
    axp.scatter([THERMAL_A[0]], [THERMAL_A[1]], facecolors="none", edgecolors="#534AB7",
                s=90, lw=1.0, zorder=4, label="locked position")
    axp.set_xlim(-150, max(x.max(), GOAL[0]) + 150)
    axp.set_ylim(-150, max(y.max(), GOAL[1]) + 150)
    axp.set_aspect("equal"); axp.grid(alpha=0.25); axp.legend(fontsize=9, loc="upper left")
    axp.set_xlabel("x (m)"); axp.set_ylabel("y (m)")
    axp.set_title(f"{case}: * = true thermal now (gone if it vanished)")
    star = axp.scatter([], [], marker="*", s=240, color="#D85A30", zorder=5)
    rad = plt.Circle(THERMAL_A, R, color="#D85A30", fill=False, ls="--", alpha=0.4)
    axp.add_patch(rad)
    trail, = axp.plot([], [], color="#185FA5", lw=1.2)
    dot, = axp.plot([], [], marker="o", ms=8, color="#185FA5")

    axh.plot(t, h, color="#185FA5", lw=1.0)
    axh.axvline(TRIGGER_T, ls=":", color="#A32D2D", lw=1, label="anomaly")
    axh.set_xlabel("time (s)"); axh.set_ylabel("altitude (m)"); axh.grid(alpha=0.25)
    axh.legend(fontsize=9, loc="best"); axh.set_title("altitude")
    hdot, = axh.plot([], [], marker="o", ms=8, color="#185FA5")
    txt = axh.text(0.03, 0.96, "", transform=axh.transAxes, va="top", fontsize=11)

    def upd(fi):
        i = idx[fi]; ti = t[i]
        if log["alive"][i]:
            star.set_offsets([[log["cx"][i], log["cy"][i]]])
            rad.center = (log["cx"][i], log["cy"][i]); rad.set_alpha(0.4)
        else:
            star.set_offsets(np.empty((0, 2))); rad.set_alpha(0.0)
        trail.set_data(x[: i + 1], y[: i + 1])
        dot.set_data([x[i]], [y[i]]); dot.set_color(mc.get(mode[i], "#185FA5"))
        hdot.set_data([t[i]], [h[i]]); hdot.set_color(mc.get(mode[i], "#185FA5"))
        txt.set_text(f"t={ti:.0f}s  h={h[i]:.0f}m\nmode: {mode[i]}")
        return star, rad, trail, dot, hdot, txt

    fig.suptitle("Locked thermal shifts / vanishes during the climb — re-acquire or give up", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    anim = FuncAnimation(fig, upd, frames=len(idx), blit=False)
    os.makedirs(_OUTPUT_DIR, exist_ok=True)
    out = os.path.abspath(os.path.join(_OUTPUT_DIR, filename))
    anim.save(out, writer=PillowWriter(fps=fps))
    plt.close(fig)
    return out


if __name__ == "__main__":
    for case, fn in (("shift", "approach_shift.gif"), ("vanish", "approach_vanish.gif")):
        log, events = run(case)
        gained = log["h"][-1] - log["h"][0]
        print(f"\n=== {case} ===  altitude {log['h'][0]:.0f} -> {log['h'][-1]:.0f} m ({gained:+.0f} m)")
        for et, ev in events:
            print(f"  t={et:5.1f}s  {ev}")
        print("saved", render(case, log, fn))
