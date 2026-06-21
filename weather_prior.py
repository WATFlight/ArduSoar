"""Weather-driven prior: real weather -> forecast thermal map -> endurance flight.

End-to-end demo of the ground-side pipeline (no onboard sensors needed):

    A) weather/collector.py   pulls real weather for a location (Open-Meteo)
    B) weather/processor.py   turns it into a thermal prior (wind, cloud_base,
                              candidate thermals) and writes prior_latest.json
    -> the endurance Engine flies that FORECAST map (instead of make_uploaded_map),
       in a world also parameterised by the same weather.

So the glider's map is no longer copied from the simulator's truth — it is an
independent guess from real weather, realistically imperfect.

    python weather_prior.py            # default: Waterloo
"""

import math

from weather.collector import fetch_weather
from weather.processor import make_prior, write_prior
from dashboard.engine import Engine, Params

LAT, LON = 43.47, -80.54                 # Waterloo, ON
BOUNDS = (-2000.0, 2000.0, -2000.0, 2000.0)


def run(lat=LAT, lon=LON, world_seed=4):
    print("A) fetching real weather ...")
    w = fetch_weather(lat, lon)
    print(f"   {w['time']}: radiation {w['radiation_wm2']:.0f} W/m2 | CAPE {w['cape_jkg']:.0f} J/kg"
          f" | BLH {w['blh_m']} m | cloud {w['cloud_pct']:.0f}% |"
          f" wind {w['wind_speed_ms']:.1f} m/s @ {w['wind_dir_deg']:.0f} deg")

    print("B) processing weather -> thermal prior ...")
    prior = make_prior(w, BOUNDS, seed=1, generated_at=w["time"])
    path = write_prior(prior)
    print(f"   wind {prior['wind']} m/s | cloud_base {prior['cloud_base_m']} m |"
          f" {prior['thermal_count']} thermals @ ~{prior['thermal_strength_ms']} m/s -> {path}")

    print("C) flying the FORECAST map (endurance) ...")
    p = Params.from_weather(prior, seed=world_seed)
    eng = Engine(p)
    n = 0
    while not eng.done and n < 70000:
        s = eng.step()
        n += 1
    print(f"   stayed aloft {s['time_aloft']/60:.1f} min | worked {s['climbs']} thermals |"
          f" battery {s['soc']*100:.0f}% | crashed={s['crashed']}")
    return prior, s


if __name__ == "__main__":
    run()
