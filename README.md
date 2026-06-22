# ArduSoar

Autonomous thermal soaring built on **ArduPilot's ArduSoar controller**
([docs](https://ardupilot.org/plane/docs/soaring.html)). See [`proposal.md`](proposal.md)
for the full direction. (Originally inspired by `sahil-kale/autoglide`; see
Attribution below.)

## Direction

We no longer build our own flight controller. ArduPilot's ArduSoar already solves
the **tactical** problem brilliantly — once the aircraft is in rising air, centre
the thermal and climb. What it can't do is know **where today's thermals are**.
That **strategic** problem is our differentiator, and it's driven by real weather.

```
 Strategic layer (this repo)              Tactical layer (ArduSoar, onboard)
 ----------------------------             ----------------------------------
 weather forecast -> thermal prior        detect lift, enter THERMAL,
 pick today's best hotspot                centre the core, climb
 fly the aircraft there (MAVLink)   -->   ArduSoar takes the handoff
```

## Asset groups

### 1. Active — the strategic differentiator
| Dir | Role |
|---|---|
| [`weather/`](weather/) | **Core.** SoaringMeteo GFS grabber + Open-Meteo Deardorff W\* pipeline → thermal-velocity / cloud-base / wind **prior**. "Where are today's thermals." |
| [`companion/`](companion/README.md) | **MAVLink bridge.** Reads the prior, picks the best reachable hotspot, flies the aircraft there, hands off to ArduSoar. **Working end-to-end in SITL.** |
| [`sitl/`](sitl/README.md) | **ArduSoar reproduction.** Drives ArduPilot SITL's ArduSoar over MAVLink with zero hardware (Milestone 1). |

### 2. Kept tooling
| Dir | Role |
|---|---|
| [`navigation/`](navigation/) | Strategic belief map + value-based commit decision. **Reused by `companion/`** (`thermal_prior.BeliefMap`, `decision.worth_climbing`). |
| [`dashboard/`](dashboard/README.md) | Plotly Dash dashboard — endurance + battery return-home. |
| [`sensors/`](sensors/README.md) | Sensor abstraction (interfaces + simulated), so guidance never touches a raw sensor. |

### 3. Baseline reference — the original self-built simulator
Superseded by ArduSoar for the onboard control loop, but **kept runnable** as a
baseline and because the dashboard still demos it:

| Dir | Role |
|---|---|
| `glider_model/` | kinematic glider (coordinated turn, bank-dependent sink) |
| `thermal_model/` | Gaussian thermal + changing-world `ThermalField` (drift / lifecycle / merge) |
| `thermal_estimator/` | rolling-window regularised least-squares thermal fit *(retired: ArduSoar's EKF replaces it)* |
| `controller/` | state machine, L1 guidance, cruise/probe/circling *(retired: ArduSoar replaces it)* |
| `simulator/`, `estimation/`, `monte_carlo/` | sim loop + plotting, state/wind fusion, robustness analysis |

The core soaring identity is shared across both worlds:

```
net climb:  h_dot  = w - v_s         (thermal lift minus sink rate)
vario:      w_meas = h_dot + v_s
thermal:    w(r)   = W_0 * exp(-r^2 / R_th^2)
```

## Run

**ArduSoar in SITL** (needs an ArduPilot SITL build + `soar-venv`; see
[`sitl/README.md`](sitl/README.md)):

```bash
sitl/run_demo.sh                 # reproduce ArduSoar thermalling in pure software
companion/run_companion_demo.sh  # weather hotspot -> fly there -> hand off to ArduSoar
```

**Weather pipeline** and **baseline simulator**:

```bash
pip install -r requirements.txt
python -m weather.openmeteo_thermal     # Deardorff W* thermal prior from Open-Meteo
python main.py                          # baseline single-thermal sim + plots
python cross_country.py                 # baseline multi-thermal cross-country
python -m pytest tests                  # unit tests (59 passing)
```

## Status

- ✅ **Milestone 1** — ArduSoar thermalling reproduced in SITL (`sitl/`).
- ✅ **Step 3** — weather-guided companion: prior → hotspot → handoff, confirmed in SITL (`companion/`).
- ☐ Hardware bring-up (Matek F405-Wing-V2 + ASPD-4525 + Pi 5); see [`docs/`](docs/).
- ☐ Multi-hotspot cross-country under live weather.

## Attribution
This project is derived from the original AutoGlide repository by Sahil Kale. The original author is not affiliated with, endorsing, collaborating on, or currently involved in this derivative project.

Original repository: [AutoGlide](https://github.com/sahil-kale/autoglide)

## Reuse
This repository may be copied, modified, and used to create derivative works for non-commercial purposes only, provided clear attribution to the original repository and author is maintained.
Such attribution must not imply endorsement, collaboration, or ongoing involvement by the original author.
