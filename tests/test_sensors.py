"""Tests for the sensor interface layer + estimation scaffolds."""

import math

from glider_model.glider import GliderState
from sensors.simulated import ground_truth_from_sim, SimulatedSensorSuite, SensorConfig, NoiseSpec
from estimation.state_fusion import PassthroughFusion
from estimation.wind_estimator import SimpleWindEstimator
from navigation.thermal_map import ThermalMap


def _truth(wind=(0.0, 0.0)):
    state = GliderState(x=100.0, y=50.0, h=500.0, heading=0.0, V=16.0, bank_angle=0.0)
    return ground_truth_from_sim(t=1.0, state=state, h_dot=1.2, wind=wind)


# --- sensors ---------------------------------------------------------------
def test_noise_free_readings_match_truth():
    cfg = SensorConfig()
    for f in ("accel", "gyro", "gps_pos", "gps_vel", "compass", "pitot", "baro_alt", "baro_vspeed", "temp", "humidity"):
        setattr(cfg, f, NoiseSpec(std=0.0))
    suite = SimulatedSensorSuite(cfg)
    snap = suite.read(_truth())
    assert abs(snap.baro.vertical_speed - 1.2) < 1e-9      # vario == h_dot
    assert abs(snap.pitot.airspeed_longitudinal - 16.0) < 1e-9
    assert abs(snap.gps.x - 100.0) < 1e-9
    assert abs(snap.compass.heading - 0.0) < 1e-9


def test_gps_ground_speed_includes_wind():
    suite = SimulatedSensorSuite(_zero_cfg())
    snap = suite.read(_truth(wind=(4.0, 0.0)))  # 4 m/s tailwind, heading +x
    assert abs(snap.gps.ground_speed - 20.0) < 1e-6       # 16 airspeed + 4 wind


def _zero_cfg():
    cfg = SensorConfig()
    for f in ("accel", "gyro", "gps_pos", "gps_vel", "compass", "pitot", "baro_alt", "baro_vspeed", "temp", "humidity"):
        setattr(cfg, f, NoiseSpec(std=0.0))
    return cfg


# --- fusion + wind ---------------------------------------------------------
def test_fusion_produces_vehicle_state():
    suite = SimulatedSensorSuite(_zero_cfg())
    fusion = PassthroughFusion()
    st = fusion.update(suite.read(_truth()))
    assert abs(st.airspeed - 16.0) < 1e-6
    assert abs(st.vario - 1.2) < 1e-6
    assert abs(st.x - 100.0) < 1e-6


def test_wind_estimator_recovers_wind():
    suite = SimulatedSensorSuite(_zero_cfg())
    fusion = PassthroughFusion()
    wind_est = SimpleWindEstimator(alpha=1.0)  # no smoothing for the test
    st = fusion.update(suite.read(_truth(wind=(3.0, -2.0))))
    w = wind_est.update(st)
    assert abs(w.wx - 3.0) < 0.2
    assert abs(w.wy - (-2.0)) < 0.2


# --- thermal map -----------------------------------------------------------
def test_map_merges_nearby_and_adds_far():
    m = ThermalMap(merge_dist=80.0)
    m.add_or_update(200, 200, 3.0, 50, t=0.0)
    m.add_or_update(210, 205, 3.4, 52, t=1.0)   # close -> merge
    m.add_or_update(900, 900, 2.5, 45, t=2.0)   # far -> new
    assert len(m.thermals) == 2
    assert m.thermals[0].n_obs == 2


def test_best_reachable_picks_by_reachability_and_score():
    m = ThermalMap()
    m.add_or_update(300, 0, 4.0, 50, t=0.0)        # strong, near
    m.add_or_update(5000, 0, 5.0, 50, t=0.0)       # stronger but far
    best = m.best_reachable(0, 0, altitude=200.0, now=0.0, glide_ratio=22.0)
    assert best is not None
    assert abs(best.x - 300.0) < 1e-6              # far one is out of glide range
