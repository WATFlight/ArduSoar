#!/usr/bin/env python3
"""Generate a weather-prior JSON (same schema as weather/prior_latest.json) whose
strongest candidate is aligned with ArduPilot SITL's built-in thermal.

SITL thermal scenario 1 (SIM_THML_SCENARI=1) places one thermal at
(north=-180 m, east=-260 m) relative to home. We put the strong candidate there
(ENU x=east=-260, y=north=-180) plus weaker/less-likely decoys elsewhere, so the
companion's strategic selection has a real choice to make. Distances are kept
inside one glide so every candidate is reachable.

This is the SITL stand-in for the real `weather/` pipeline output: in the field
the companion reads the live prior instead.
"""
import json
import os

# [x_east_m, y_north_m, W*_strength_ms, probability]
CANDIDATES = [
    [-260.0, -180.0, 4.0, 0.90],   # <-- aligned with the SITL thermal (the answer)
    [320.0,  120.0, 2.4, 0.55],    # decoy: weaker, other direction
    [-40.0,  410.0, 3.1, 0.35],    # decoy: medium strength, low probability
    [500.0, -450.0, 3.6, 0.20],    # decoy: strong-ish but unlikely and far
]

prior = {
    "generated_at": "SITL",
    "location": {"lat": -35.363261, "lon": 149.165230},  # CMAC; companion overrides with live home in --origin home
    "bounds": [-2000.0, 2000.0, -2000.0, 2000.0],
    "wind": [0.0, 0.0],
    "cloud_base_m": 2000,
    "thermal_strength_ms": 4.0,
    "thermal_count": len(CANDIDATES),
    "candidates": CANDIDATES,
}

if __name__ == "__main__":
    out = os.path.join(os.path.dirname(__file__), "sitl_prior.json")
    with open(out, "w") as f:
        json.dump(prior, f, indent=2)
    print(f"wrote {out} ({len(CANDIDATES)} candidates, strong one at ENU (-260,-180))")
