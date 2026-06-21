"""Tests for the thermal estimator and the glider model."""

import math

from config import DT
from glider_model.glider import GliderState, SimpleGlider
from thermal_model.thermal import GaussianThermal
from thermal_estimator.estimator import ThermalEstimator


# --- glider model -----------------------------------------------------------
def _glider(bank=0.0):
    return SimpleGlider(GliderState(0.0, 0.0, 300.0, 0.0, 16.0, bank))


def test_zero_thermal_descends():
    g = _glider()
    h_dot = g.step(bank_command=0.0, thermal_lift=0.0, dt=0.1)
    assert h_dot < 0.0


def test_strong_thermal_climbs():
    g = _glider()
    h_dot = g.step(bank_command=0.0, thermal_lift=3.0, dt=0.1)
    assert h_dot > 0.0


def test_higher_bank_increases_sink():
    level = _glider(bank=0.0).sink_rate()
    banked = _glider(bank=math.radians(40.0)).sink_rate()
    assert banked > level


# --- estimator --------------------------------------------------------------
def test_too_few_measurements_returns_none():
    est = ThermalEstimator()
    est.add_measurement(0.0, 0.0, 0.5, 0.7)
    assert est.estimate() is None


def test_recovers_known_thermal():
    """Feed synthetic samples from a known thermal and check recovery."""
    true = GaussianThermal(x_c=200.0, y_c=200.0, W_0=3.5, R_th=50.0)
    est = ThermalEstimator()

    # Sample a small grid covering the core; w_meas = h_dot + sink with the
    # identity h_dot = w - sink, so the stored lift equals the true w.
    sink = 0.7
    for i in range(7):
        for j in range(7):
            x = 160.0 + i * 12.0
            y = 160.0 + j * 12.0
            w = true.vertical_velocity(x, y)
            est.add_measurement(x, y, h_dot=w - sink, sink_rate=sink)

    out = est.estimate()
    assert out is not None
    assert abs(out.x_c - 200.0) < 15.0
    assert abs(out.y_c - 200.0) < 15.0
    assert abs(out.W_0 - 3.5) < 1.0
    assert abs(out.R_th - 50.0) < 20.0


# --- drifting thermal (small-goal #1: follow the moving core) ---------------
def _feed_drifting_window(est, c0, wind, W0=3.5, R=50.0, sink=0.7):
    """Emulate the glider sampling a small grid that rides along with a thermal
    whose core drifts as ``c(t) = c0 + wind * t``. One ~5 s window of samples."""
    offs = [-36.0, -24.0, -12.0, 0.0, 12.0, 24.0, 36.0]
    i = 0
    for ox in offs:
        for oy in offs:
            t = (i + 1) * DT
            cx, cy = c0[0] + wind[0] * t, c0[1] + wind[1] * t
            r2 = ox * ox + oy * oy
            w = W0 * math.exp(-r2 / R ** 2)          # lift from the TRUE moving core
            est.add_measurement(cx + ox, cy + oy, h_dot=w - sink, sink_rate=sink)
            i += 1
    return i * DT                                     # time of the last sample


def test_tracks_drifting_thermal_when_told_the_wind():
    """With set_wind(), the estimate sits on the core's CURRENT (drifted)
    position; ignoring the wind makes it lag upwind."""
    wind = (4.0, -2.0)
    c0 = (100.0, -50.0)

    est_w = ThermalEstimator()
    est_w.set_wind(wind)                              # fit in the wind-moving frame
    t_now = _feed_drifting_window(est_w, c0, wind)
    out_w = est_w.estimate()

    est_n = ThermalEstimator()                        # assumes a static thermal
    _feed_drifting_window(est_n, c0, wind)
    out_n = est_n.estimate()

    assert out_w is not None and out_n is not None
    true_now = (c0[0] + wind[0] * t_now, c0[1] + wind[1] * t_now)
    err_w = math.hypot(out_w.x_c - true_now[0], out_w.y_c - true_now[1])
    err_n = math.hypot(out_n.x_c - true_now[0], out_n.y_c - true_now[1])
    assert err_w < 8.0                                # tracks the drifted core
    assert err_w < err_n                              # and clearly beats ignoring drift


def test_wind_object_accepted_by_set_wind():
    """set_wind() takes either a (wx, wy) tuple or a Wind-like object."""
    class _W:
        wx, wy = 3.0, -1.0
    est = ThermalEstimator()
    est.set_wind(_W())
    assert est.wind == (3.0, -1.0)
