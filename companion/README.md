# Weather-guided MAVLink companion (step 3)

**The project differentiator.** ArduPilot's ArduSoar is excellent at the *tactical*
problem — once the aircraft is in rising air, centre the thermal and climb. It has
no idea *where* today's thermals are. That's the *strategic* problem, and it's
what our weather pipeline answers. This companion is the bridge: it reads the
forecast, picks the best reachable hotspot, flies the aircraft there over MAVLink,
and hands off to ArduSoar.

```
 Strategic layer (this companion)        Tactical layer (ArduSoar, onboard)
 --------------------------------         ----------------------------------
 weather prior  -> best hotspot           detect lift, enter THERMAL,
 fly there (autopilot mission)            centre the core, climb
 enable soaring + hand off          -->   ArduSoar takes it from here
 confirm / disconfirm the candidate <--   (did we actually find lift?)
```

It reuses the repo's existing strategic brain — `navigation.thermal_prior.BeliefMap`
(reachability + goal-progress scoring, confirm/disconfirm) — so the "where are
today's thermals" logic is shared with the offline simulator.

## End-to-end result (SITL)

```
Chosen hotspot: ENU (-260,-180) m  W*=4.0 p=0.90  -> -35.364879,149.162373
--> Reached hotspot (89 m), handing off to ArduSoar
AP: Soaring: Enabled, automatic mode changes.
AP: Soaring: Thermal detected, entering Thermal
--> ArduSoar entered THERMAL at 123 m
--> Climbed +60 m in the thermal (now 183 m)
RESULT: CONFIRMED
  candidate prob after flight: 1.00
```

## Run

```bash
companion/run_companion_demo.sh
```

It generates a SITL-aligned weather prior, launches a fresh `plane-soaring` SITL,
runs the companion, and tears down. Needs the same `../../ardupilot` build and
`../../soar-venv` as `sitl/` (see `../sitl/README.md`).

## Files

| File | Role |
|---|---|
| `weather_guided_companion.py` | orchestrator: select hotspot → fly there → hand off → confirm |
| `mav.py` | pymavlink helpers (connect, params, modes, arm, mission, soaring aux, GUIDED goto) |
| `geo.py` | local ENU metres ↔ lat/lon |
| `make_sitl_prior.py` | generates a weather prior aligned to SITL's built-in thermal |
| `sitl_prior.json` | generated SITL stand-in for the real `weather/` pipeline output |
| `run_companion_demo.sh` | one-shot SITL + companion orchestrator |

## Design notes / gotchas

- **SITL/weather alignment.** SITL's built-in thermal (scenario 1) sits at
  (north −180 m, east −260 m) from home. `make_sitl_prior.py` puts the strong
  candidate exactly there (plus weaker decoys, so selection is non-trivial). In
  the field, the companion reads the live `weather/` prior instead; pass
  `--origin prior` to interpret candidate metres relative to `prior.location`
  rather than the live home.
- **Navigation = AUTO mission, not GUIDED.** ArduPlane GUIDED ignored a raw
  `SET_POSITION_TARGET_GLOBAL_INT` (it just loitered where it entered GUIDED),
  and `MAV_CMD_DO_REPOSITION` didn't steer reliably mid-flight here either. A
  dynamically-built AUTO mission (takeoff → waypoint@hotspot →
  `NAV_LOITER_UNLIM`@hotspot) flies to the hotspot reliably. `goto_global()` (a
  DO_REPOSITION helper) is kept in `mav.py` for future GUIDED use.
- **Hand off in AUTO, not FBWB.** Stay in AUTO loitering at the hotspot so the
  aircraft keeps circling *inside* the thermal while ArduSoar detects lift and
  switches to THERMAL. FBWB flies straight out of the thermal and the contact is
  lost immediately.
- **Enabling soaring** uses `MAV_CMD_DO_AUX_FUNCTION(88, HIGH)` — see
  `../sitl/README.md` for why a plain RC override doesn't work headless.

## Driving the prior from live weather

`weather/soaringmeteo_prior.py` builds a companion-schema prior from a live
SoaringMeteo forecast (W\* → candidate strength, soaring-layer top → ceiling, BL
wind → drift):

```bash
python -m weather.soaringmeteo_prior        # fetch today's run -> prior JSON
```

`weather/openmeteo_prior.py` does the same from Open-Meteo's Deardorff W\*, so both
sources feed one prior entry point. The companion's selection runs straight on it —
real weather → best reachable hotspot at a real lat/lon. (The SITL fly-out still
uses `make_sitl_prior.py`, since SITL's synthetic thermal sits at a fixed offset;
on real hardware the companion flies the forecast coordinates directly.)

## Cross-country relay (multi-hotspot)

`weather_guided_xc.py` hops between forecast thermals toward a goal: pick the best
reachable candidate → fly there → hand off → **confirm** (climbed) or **disconfirm**
(no lift) in the belief map → hop to the next.

```bash
companion/run_xc_demo.sh
```

SITL has a single synthetic thermal, so only the aligned hop climbs; the others
demonstrate the fly → search → disconfirm → re-target machinery. Example run:
`hop 1 CONFIRMED (+40 m), hop 2 disconfirmed, hop 3 disconfirmed`. A true
multi-climb cross-country is best watched in the **dashboard** on real weather
(`python -m dashboard.app` → 🌤 use weather), or on hardware.
