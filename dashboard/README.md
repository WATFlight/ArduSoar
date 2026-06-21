# ArduSoar dashboard (Plotly Dash)

Interactive front-end for the thermal-soaring simulation: input parameters,
watch the flight live, and adjust playback speed to observe.

## Run

```bash
pip install dash plotly        # one-time (see requirements.txt)
cd ardusoar          # the project root
python -m dashboard.app         # open http://127.0.0.1:8050
```

## Layout

- **Left panel — inputs & controls**
  - `▶ Play / ⏸ Pause / ⏮ Reset`
  - **playback speed** — sim-ticks run per frame (1×–50×); slow-mo to study a
    catch, fast-forward through cruise. (This is observation speed, separate from
    the aircraft's airspeed.)
  - **wind x/y**, **airspeed**, **battery (Wh)**, **map decay τ**, **seed**,
    **sensor noise** — changes apply on **Reset** (they rebuild the world).
- **Right panel — live visualisation**
  - **2-D map**: drifting/meandering thermals (stars), uploaded map points
    coloured by survey status (blue=unsurveyed, green=lift, grey=empty,
    tan=written off), glider trail + position (coloured by mode), home, wind arrow.
  - **scrolling altitude**: last 10 min, shaded where the electric sustainer ran.
  - **gauges**: battery % and home-reach fuel margin.

## How it's wired

```
dashboard/engine.py   Params + Engine.step()   <- headless, one tick per call
dashboard/app.py      Dash UI; dcc.Interval -> Engine.step() x speed -> figures
```

`Engine` mirrors `explore.py`'s per-tick logic but is parameterised (no module
globals), so every slider maps onto a `Params` field. The same engine is the
clean "edge / tactical layer" seam for a future cloud split.

## Fly today's real forecast

The **🌤 use weather** button fetches a real forecast for the (lat, lon) and flies
it in sim — the glider's map *is* the forecast (no cheat). Pick the source:

- **Open-Meteo (rad/CAPE)** — bulk params → strength heuristic (`weather/processor.py`)
- **Open-Meteo W\* (Deardorff)** — convective velocity (`weather/openmeteo_prior.py`)
- **SoaringMeteo (GFS)** — thermal velocity / BL top / wind (`weather/soaringmeteo_prior.py`)

This is where you watch the **weather-guided strategy** (prior → hop between
hotspots → confirm/disconfirm → reach the goal) play out on real data. The MAVLink
companion (`../companion/`) runs the same strategy against a real autopilot.
