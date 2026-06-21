"""Tests for the SoaringMeteo point-forecast grabber.

Pure logic (coordinate math, parsing) runs offline; one integration test does a
live fetch to confirm the endpoint + parsing actually work end to end.
"""

import math
import urllib.error

import pytest

from weather import soaringmeteo as sm

# north-america zone as published in the index
NA_ZONE = {"id": "north-america",
           "raster": {"proj": "EPSG:4326", "resolution": 0.25,
                      "extent": [-124.125, 16.875, -69.875, 51.625]}}


def test_grid_index_matches_app_formula():
    # Oklahoma (36.687, -97.137) -> col 107, row 59 (col=round((lon-x0)/res-.5), etc.)
    col, row = sm.grid_index(36.687, -97.137, NA_ZONE)
    assert (col, row) == (107, 59)


def test_zone_for_picks_covering_zone():
    index = {"zones": [NA_ZONE,
                       {"id": "europe", "raster": {"resolution": 0.25,
                        "extent": [-25.625, 27.375, 39.625, 70.125]}}]}
    assert sm.zone_for(36.687, -97.137, index)["id"] == "north-america"
    with pytest.raises(ValueError):
        sm.zone_for(0.0, 0.0, index)            # ocean, no zone


def test_extract_hourly_uses_app_field_mapping():
    # one grid cell, one day, two hours — mirrors the real schema
    point = {"h": 313, "d": [{"th": 4, "h": [
        {"t": "2026-06-20T15:00:00Z", "v": 11, "bl": {"h": 613, "u": 1, "v": 15}},
        {"t": "2026-06-20T18:00:00Z", "v": 28, "bl": {"h": 1800, "u": -10, "v": 0}},
    ]}]}
    rows = sm.extract_hourly(point)
    assert len(rows) == 2
    r0 = rows[0]
    assert r0["thermal_velocity_ms"] == 1.1           # v / 10
    assert r0["soaring_layer_top_m"] == 613           # bl.h
    assert r0["wind_bl_speed_kmh"] == round(math.hypot(1, 15), 1)
    # second hour: wind blows toward -x (west), so it comes FROM the east (~90 deg)
    assert rows[1]["thermal_velocity_ms"] == 2.8
    assert rows[1]["wind_bl_from_deg"] == 90


def test_point_lonlat_inverts_grid_index():
    # grid_index -> point_lonlat should land back within half a cell of the input
    col, row = sm.grid_index(36.687, -97.137, NA_ZONE)
    lon, lat = sm.point_lonlat(col, row, NA_ZONE)
    assert abs(lon - (-97.137)) <= 0.125 and abs(lat - 36.687) <= 0.125


def test_hour_at_and_peak_time():
    point = {"d": [{"h": [
        {"t": "2026-06-20T15:00:00Z", "v": 11, "bl": {"h": 600}},
        {"t": "2026-06-20T18:00:00Z", "v": 28, "bl": {"h": 1800}},
    ]}]}
    assert sm.peak_time(point) == "2026-06-20T18:00:00Z"          # strongest thermal
    assert sm._hour_at(point, "2026-06-20T15:00:00Z")["v"] == 11
    assert sm._hour_at(point, "1999-01-01T00:00:00Z") is None


@pytest.mark.integration
def test_live_fetch_region_oklahoma():
    """Sample a small box and check it returns a real 2-D grid at one time."""
    try:
        meta, recs = sm.fetch_region(35.5, 37.5, -98.5, -96.5)   # ~2x2 deg
    except (urllib.error.URLError, RuntimeError) as e:
        pytest.skip(f"network/data unavailable: {e}")
    assert recs, "no region records"
    assert meta["time"] in meta["available_times"]
    lats = {r["lat"] for r in recs}
    lons = {r["lon"] for r in recs}
    assert len(lats) >= 2 and len(lons) >= 2, "expected a 2-D grid of points"
    tv = [r["thermal_velocity_ms"] for r in recs]
    assert all(0.0 <= x <= 8.0 for x in tv), "thermal velocity out of range"


@pytest.mark.integration
def test_live_fetch_oklahoma():
    """Actually grab SoaringMeteo data and sanity-check it."""
    try:
        meta, rows = sm.fetch_table(36.687, -97.137)
    except (urllib.error.URLError, RuntimeError) as e:
        pytest.skip(f"network/data unavailable: {e}")
    assert meta["zone"] == "north-america"
    assert rows, "no hourly rows returned"
    # values must be physically plausible
    tv = [r["thermal_velocity_ms"] for r in rows]
    blt = [r["soaring_layer_top_m"] for r in rows if r["soaring_layer_top_m"] is not None]
    assert all(0.0 <= x <= 8.0 for x in tv), f"thermal velocity out of range: {max(tv)}"
    assert blt and all(0 <= x <= 6000 for x in blt), "soaring-layer top out of range"
    assert max(tv) > 0.0, "no positive thermal velocity anywhere in the forecast"
