"""Tests for the Open-Meteo w* (thermal velocity) computation."""

import math
import urllib.error

import pytest

from weather import openmeteo_thermal as om


def test_wstar_zero_when_not_convective():
    assert om.compute_wstar(-30.0, 1500, 25.0, 970.0) == 0.0     # downward flux (night)
    assert om.compute_wstar(0.0, 1500, 25.0, 970.0) == 0.0
    assert om.compute_wstar(200.0, 0.0, 25.0, 970.0) == 0.0      # no boundary layer


def test_wstar_matches_hand_calc():
    # the Oklahoma 18:00Z sample: H=194.4, blh=1475, T=33.4C, P=970.1 hPa
    w = om.compute_wstar(194.4, 1475.0, 33.4, 970.1)
    # hand calc: rho=P/(Rd T)=1.10, Q0=H/(rho cp)=0.176, w=(g/T*Q0*zi)^(1/3)~2.0
    assert abs(w - 2.0) < 0.15


def test_wstar_grows_with_heat_and_depth():
    base = om.compute_wstar(150, 1200, 30, 970)
    assert om.compute_wstar(300, 1200, 30, 970) > base           # more heating
    assert om.compute_wstar(150, 2400, 30, 970) > base           # deeper layer


def test_wind_uv_direction():
    # wind FROM the south (180 deg) blows toward the north: +v, ~0 u
    u, v = om._wind_uv(36.0, 180.0)
    assert abs(u) < 0.01 and v > 0


def test_axis_snaps_to_grid():
    ax = om._axis(34.0, 35.0, 0.25)
    assert ax[0] == 34.0 and abs(ax[1] - ax[0] - 0.25) < 1e-9 and 35.0 in ax


@pytest.mark.integration
def test_live_region_oklahoma():
    try:
        meta, recs = om.fetch_region(36.0, 37.0, -98.0, -97.0, "2026-06-21T18:00:00Z")
    except (urllib.error.URLError, RuntimeError) as e:
        pytest.skip(f"network unavailable: {e}")
    assert recs, "no records"
    lats = {r["lat"] for r in recs}
    lons = {r["lon"] for r in recs}
    assert len(lats) >= 2 and len(lons) >= 2
    tv = [r["thermal_velocity_ms"] for r in recs]
    assert all(0.0 <= x <= 8.0 for x in tv)
    assert all(r["soaring_layer_top_m"] is None or r["soaring_layer_top_m"] >= 0 for r in recs)
