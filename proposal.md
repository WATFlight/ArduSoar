# ArduSoar — Autonomous Thermal-Soaring Simulation & Research Platform

## 0. Goal

Build a Python simulation and research platform for **autonomous thermal soaring**,
aligned with **ArduPilot's ArduSoar controller**
([ardupilot.org/plane/docs/soaring.html](https://ardupilot.org/plane/docs/soaring.html)).

The aim is no longer a standalone toy glider — it is to **reproduce, study, and extend
the ArduSoar approach in software**, so we can iterate on estimation/guidance fast,
drive it with real weather, and ultimately **validate against ArduPilot SITL** and
hardware.

ArduSoar (Tabor, Guilliard, Kolobov, 2018 — *"ArduSoar: an Open-Source Thermalling
Controller for Resource-Constrained Autopilots"*) is the soaring feature shipped in
ArduPilot Plane. It lets a powered glider **cut the motor, glide, detect rising air,
and circle thermals** to extend endurance, returning to powered flight or RTL when it
sinks too low.

---

## 1. How ArduSoar works (the reference design)

| Stage | ArduSoar | What it does |
|---|---|---|
| **Sink/lift sensing** | drag-polar (`SOAR_POLAR_K/B/CD0`) + airspeed | expected sink vs measured descent → estimate of **air vertical speed** |
| **Thermal trigger** | `SOAR_VSPEED` (default 0.7 m/s) | rising air above threshold → enter thermalling |
| **Thermal estimation** | EKF over a **Wharington Gaussian thermal** (strength, radius, x, y) | online estimate of the thermal core from the variometer + GPS track |
| **Circling** | switch to **LOITER**, recenter on the estimate | climb in the core |
| **Altitude bands** | `SOAR_ALT_CUTOFF` / `SOAR_ALT_MIN` / `SOAR_ALT_MAX` | when to glide, when to motor, when to leave the thermal |
| **Energy / return** | motor cutoff when gliding; RTL / climb below `SOAR_ALT_MIN`; `SOAR_MAX_DRIFT` | stay reachable; don't drift too far from home |
| **Hysteresis** | `SOAR_MIN_THML_S`, `SOAR_MIN_CRSE_S` | minimum thermal/cruise durations to avoid mode chatter |
| **Speed-to-fly** | MacCready-style | choose cruise airspeed for the conditions |

The control law is a **supervisory layer**: glide (zero-throttle AUTO/CRUISE/FBWB)
↔ circle (LOITER), bounded by altitude and distance-from-home.

---

## 2. Our simulation already mirrors ArduSoar

What we have built maps almost 1:1 onto ArduSoar — this is the basis for studying and
extending it:

| ArduSoar concept | Our implementation |
|---|---|
| drag-polar sink from airspeed/bank | `glider_model/` kinematic glider: `sink = base_sink/cos(phi)` |
| variometer → air vertical speed | `w_meas = h_dot + sink` (the core identity) |
| Wharington Gaussian thermal | `thermal_model/` Gaussian `w(r)=W0·exp(-r²/R²)` (+ lifecycle, drift, merge) |
| online thermal estimate | `thermal_estimator/` windowed nonlinear least-squares (EKF is the next step) |
| `SOAR_VSPEED` trigger | Cruise/Probe/Thermal state machine thresholds |
| LOITER circling + recenter | `controller/circling_control.py` |
| `SOAR_ALT_*` bands, motor cutoff, RTL | endurance + **return-home** logic + **electric sustainer** (battery) |
| `SOAR_MAX_DRIFT` | home-range / return-home guard |
| MacCready speed-to-fly | `navigation/decision.py` `worth_climbing` |

We also go **beyond** the stock feature where it helps research: a real **weather
pipeline** (Open-Meteo / SoaringMeteo → thermal-velocity W\* priors), a live **dashboard**,
and a **changing-world** thermal model (drift, lifecycle, merging).

---

## 3. The physics (unchanged core)

```text
net climb:        h_dot = w - v_s
variometer:       w_meas = h_dot + v_s
Gaussian thermal: w(r) = W0 · exp(-r² / R²)
coordinated turn: heading_dot = g·tan(phi)/V,   v_s = base_sink / cos(phi)
```

The estimator fits `(x_c, y_c, W0, R)` to a rolling window of `(x, y, w_meas)` — the
same quantity ArduSoar's EKF tracks, just with a different filter.

---

## 4. Roadmap toward ArduPilot

**Phase 1 — Python sim (done / ongoing).** ArduSoar-equivalent stack: variometer,
Gaussian estimator, Cruise/Probe/Thermal state machine, L1 + circling, energy &
return-home, electric sustainer, weather-driven W\* priors, dashboard, Monte-Carlo
tests. Approaches realism through scenario testing.

**Phase 2 — Align to ArduSoar semantics.**
- Adopt the `SOAR_*` parameter names/meaning (`SOAR_VSPEED`, `SOAR_ALT_MIN/CUTOFF/MAX`,
  `SOAR_MAX_DRIFT`, `SOAR_MIN_THML_S/CRSE_S`, `SOAR_POLAR_*`).
- Swap the windowed least-squares estimator for an **EKF over the Wharington thermal**
  (exactly ArduSoar's filter) and compare the two.
- Use the published drag-polar so sink matches a real airframe.

**Phase 3 — ArduPilot SITL.**
- Run the **real ArduSoar** in ArduPilot SITL, driving it with our **changing-world /
  weather-derived thermal field** (custom SITL thermal model).
- Compare our Python controller against stock ArduSoar on the same thermals.
- Feed **real-weather W\*** (Open-Meteo / SoaringMeteo) as the pre-flight thermal map.

**Phase 4 — Hardware.** Run ArduSoar on an ArduPilot autopilot in a real powered
glider; close the loop against real thermals (our onboard estimation/return-home as a
research overlay).

---

## 5. Non-goals (for now)

- Full 6-DOF aircraft / actuator dynamics (kinematic is enough for guidance research).
- Reinforcement learning.
- Re-implementing all of ArduPilot — we target the **soaring** subsystem only.

---

## 6. Current capabilities (snapshot)

- Variometer-based **Gaussian thermal estimation** (least-squares; wind-frame drift tracking).
- **Cruise / Probe / Thermal** state machine + L1 guidance + off-center circling.
- **Changing world**: thermal lifecycle (grow/hold/decay), wind drift + per-thermal
  meander, merging thermals, imperfect uploaded maps.
- **Energy management**: electric sustainer (battery base + motor drain), battery-aware
  **return-home** (RETURN_HOME → MISSION_COMPLETE), à la `SOAR_ALT_*` + `SOAR_MAX_DRIFT`.
- **Weather pipeline**: SoaringMeteo grabber + Open-Meteo **W\*** (Deardorff convective
  velocity) → thermal-velocity priors and 2-D maps.
- **Dashboard** (Plotly Dash) and **Monte-Carlo / unit tests**.
- Runs on a Raspberry Pi 5.

---

## 7. References

- ArduPilot Soaring docs — <https://ardupilot.org/plane/docs/soaring.html>
- ArduSoar paper — Tabor, Guilliard, Kolobov, *"ArduSoar: an Open-Source Thermalling
  Controller for Resource-Constrained Autopilots"*, IROS 2018 — <https://arxiv.org/abs/1802.08215>
- Wharington thermal model / MacCready theory (classical soaring).
