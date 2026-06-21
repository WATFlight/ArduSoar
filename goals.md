# ArduSoar — Detailed Goals

This document defines what "done" means as the project moves from an idealised
single-thermal demo toward **autonomous cross-country soaring in a realistic,
changing environment**, validated in simulation and ready to port to hardware.

It complements [`proposal.md`](proposal.md) (the original step-by-step build) and
records the current status of each goal so we know what to do next.

---

## 0. North star

A glider that, given only a rough pre-flight thermal map + wind forecast and its
own onboard sensors, **soars cross-country to a goal across a field of thermals
that appear, drift, weaken, and die** — choosing what to climb, what to skip, and
where to search, and degrading gracefully when the map is wrong.

---

## 1. Realistic world model  *(NEXT PRIORITY)*

The current world is idealised in four ways; this goal replaces it. Implement as
a new `random_field` model, leaving the corridor model intact.

### 1.1 Random 2-D thermal field
- Thermals placed at **random positions over the whole map** (Poisson / uniform
  with a minimum spacing), **not** along a corridor.
- **Acceptance:** thermals fill a 2-D area; no assumption that lift sits near the
  start→goal line.

### 1.2 Stochastic birth–death (space–time point process)
- New thermals **spawn at random times and places** (Poisson rate per area·time);
  each gets a random lifespan with the trapezoid strength envelope; old ones die.
- The population **continuously turns over** — at any instant some are growing,
  some peaking, some dying, and the set is never fixed.
- **Acceptance:** the count and locations of live thermals vary through the flight;
  thermals exist that were *not* present at takeoff.

### 1.3 Wind-driven drift
- Each thermal's position **advects downwind over its life**:
  `pos(t) = birth_pos + wind · (t − birth)`.
- **Acceptance:** a thermal's location at altitude/time differs from its birth
  location by the integrated wind.

### 1.4 Imperfect uploaded map
- The pre-flight map is a **snapshot at upload time** with:
  (a) a **global registration offset** (and optionally small rotation/scale),
  (b) **per-thermal position noise**,
  (c) **staleness** — thermals that have since died/moved/been born differ.
- **Acceptance:** the algorithm must succeed despite a *systematically shifted*
  and partially wrong map, not just per-point jitter.

> Net effect: the uploaded map is only a **rough prior over a churning random
> field**. Corridor-style figure-8 marching no longer suffices — the glider must
> navigate and search in true 2-D and lean heavily on online sensing.

---

## 2. Guidance & search capabilities

| Capability | Status |
|---|---|
| Online Gaussian thermal estimator (windowed least squares) | ✅ done |
| Cruise / Probe / Thermal state machine | ✅ done |
| L1 guidance + circling control | ✅ done |
| Figure-8 search (find off-route thermals) | ✅ done |
| Capture hysteresis (no drive-by misses) | ✅ done |
| Cloud-base departure + thermal hopping (cross-country) | ✅ done |
| Bounded **expanding** search + map fallback (energy-aware) | ✅ done |
| Prior-guided search (fly to likely points, validate, confirm/disconfirm) | ✅ done |
| Belief decay + dead-arrival handling | ✅ done |
| Online thermal map (merge / mark-dead) + value decision (MacCready) | ✅ done |
| **2-D search/replanning over a churning random field** | ⬜ goal 1 unlocks this |
| **Opportunistic capture proven in 2-D (grab off-map new thermals)** | ⬜ partial (coded, not yet exercised) |
| **Robustness to a globally-offset map** | ⬜ goal 1.4 |

---

## 3. Estimation & robustness  *(KEY GAP)*

- **3.1 Chi-squared confidence** — replace `confidence = 1/(1+mse)`, which is
  near-binary and gets fooled by circling noise into false captures. Needed
  before anything runs on **real (noisy) sensor data**. *(Today the prior-guided
  and lifecycle demos run on clean sensors because of this.)*
  **Acceptance:** prior-guided search succeeds with realistic vario noise, with
  no false captures in dead air.
- **3.2 State fusion** — a real EKF/AHRS behind the `StateFusion` interface
  (today a pass-through). **Acceptance:** stable `VehicleState` under GPS dropout
  + IMU/baro noise.
- **3.3 Wind estimation** — basic version done; refine (filtered / least-squares)
  and feed the map's drift correction (goal 1.3).

---

## 4. Sensors & hardware path

- **4.1 Sensor abstraction** — guidance reads `VehicleState / Wind / ThermalMap`,
  never raw sensors. ✅ done (`sensors/`, simulated implementations).
- **4.2 SITL** — drive the Python brain against **ArduPlane SITL over MAVLink**
  (no hardware). First real-interface milestone.
- **4.3 Companion-computer architecture** — Python brain on a Pi 5; a dedicated
  flight controller (ArduPilot/PX4) closes the inner loop. **Do not rewrite in C**
  to run on the Pi; port to C only if/when moving onto a bare MCU.
- **4.4 HIL → first flight** — bench test, then a real glider.

---

## 5. Testing & validation

- **5.1 Lifecycle / random-field Monte Carlo** — many flights over randomised
  fields, spawn rates, lifespans, wind, and **map staleness/offset**.
- **5.2 Metrics** — goal-reach rate, steady-state altitude margin, # dead-arrivals,
  energy spent searching, vs a **static-map baseline**.
- **5.3 Expose failure boundaries** — find where the algorithm breaks (as the
  original `monte_carlo/` did for the static case).

---

## 6. Success criteria (measurable)

1. On a **random 2-D churning field** with a **globally-offset, stale map**, the
   glider reaches the goal in ≥ X % of randomised runs (target to be set after
   the first Monte Carlo baseline).
2. It **does not commit** to dead/weak thermals when high, and **survives** on
   marginal lift when low.
3. It runs on **realistic sensor noise** (goal 3.1) without false captures.
4. The same guidance code runs unchanged against **ArduPlane SITL** (goal 4.2).

---

## 7. Non-goals (for now)

- Full 6-DoF aircraft dynamics / actuator modelling (the flight controller owns
  inner-loop stabilisation).
- Vision-based thermal detection (camera) — interface left open, not implemented.
- Rewriting the guidance stack in C for the Pi (see goal 4.3).
- Thermal-strength *forecasting* before entry beyond the simple prior.
