# Ground path planner (our scope)

Per the team split, **we own the ground-side path planning**: turn today's weather
into an ordered route of thermal waypoints toward a goal, and export it as an
**uploadable path**. The Pi 5 interprets the uploaded path (+ vision, + returns
data) and the flight controller flies it — neither is built here.

```
weather prior  ->  plan_route()  ->  ordered lat/lon waypoints  ->  route.json + route.waypoints
   (ours)          greedy chain        (the path we hand off)         (Pi 5 uploads this)
```

## How it plans

Greedy strategic chain reusing `navigation.thermal_prior.BeliefMap` (same scoring
as the dashboard and companion): from home, pick the best reachable candidate
toward the goal, step to it, repeat until the goal vicinity or candidates run out.

- **Local box** (`--prior` or `--source ... --lat --lon`): candidates are a sampled
  field in a small ±2 km box → usually one best thermal (you can glide anywhere
  locally, so a single waypoint is the right answer).
- **Cross-country** (`--region-km N`): candidates are the **real W\* grid cells**
  over an N-km box → a genuine multi-waypoint route that hops thermal-to-thermal.

## Run

```bash
# cross-country route over a 150 km box from live SoaringMeteo
python -m planner.route_planner --source soaringmeteo --lat 43.47 --lon -80.54 --region-km 150

# local best-thermal route, or from a saved prior, or toward a chosen goal
python -m planner.route_planner --source openmeteo --lat 43.47 --lon -80.54
python -m planner.route_planner --prior weather/data/soaringmeteo_prior_43.47_-80.54.json
python -m planner.route_planner --source soaringmeteo --lat 43.47 --lon -80.54 \
       --region-km 150 --goal-lat 44.2 --goal-lon -79.5
```

Outputs land in `planner/routes/` (gitignored):
- `route_*.json` — our rich format: each waypoint with lat/lon + ENU + forecast W\* + probability, plus goal / wind / cloud base.
- `route_*.waypoints` — **standard QGC WPL 110** (home, takeoff, hotspot waypoints) the Pi 5 / Mission Planner can upload directly.

## The hand-off interface (decided)

Flight controller is **ArduPilot** (confirmed), so the path is shipped in
ArduPilot's native language — the Pi 5 uploads it with zero translation:

1. **`route_*.waypoints` — a native ArduPilot mission (QGC WPL 110).** The soaring
   strategy is encoded *in the mission*: `NAV_TAKEOFF`, then each hotspot is a
   `NAV_LOITER_TO_ALT` (circle/soar up to `ceiling_alt_m`, then glide on), ending
   in `RETURN_TO_LAUNCH`. With `SOAR_ENABLE=1` the loiters are where ArduSoar works
   the thermal; no lift → it climbs on the motor, so it never hangs.
2. **`route_*.json` — sidecar** with what the mission format can't carry: per-waypoint
   forecast W\* / probability, goal, wind, cloud base, and a `handoff` block listing
   the Pi 5 steps.

**Pi 5 steps** (in `route.json` → `handoff`): set `SOAR_ENABLE=1`, enable soaring
via `MAV_CMD_DO_AUX_FUNCTION(88, HIGH)` once airborne, upload the `.waypoints`, set
mode `AUTO`, arm. Then it does vision and returns data; if it reports
vision-confirmed thermal positions we feed them back (`BeliefMap.confirm/disconfirm`)
and re-emit an updated route.

## Validated in SITL

`sitl/run_route_demo.sh` plans a SITL-local route and flies that exact `.waypoints`
mission: it uploads to ArduPilot cleanly, flies the planned route, and ArduSoar
engages a thermal at the planned hotspot — proving the path we hand off is a valid,
flyable ArduPilot mission.
