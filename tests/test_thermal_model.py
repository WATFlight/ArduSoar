"""Tests for the Gaussian thermal model."""

from thermal_model.thermal import GaussianThermal


def test_center_has_max_lift():
    th = GaussianThermal(x_c=10.0, y_c=-5.0, W_0=3.0, R_th=40.0)
    assert th.vertical_velocity(10.0, -5.0) == 3.0


def test_far_away_has_low_lift():
    th = GaussianThermal(x_c=0.0, y_c=0.0, W_0=3.0, R_th=40.0)
    assert th.vertical_velocity(1000.0, 1000.0) < 0.01


def test_lift_decreases_with_distance():
    th = GaussianThermal(x_c=0.0, y_c=0.0, W_0=3.0, R_th=40.0)
    near = th.vertical_velocity(10.0, 0.0)
    far = th.vertical_velocity(60.0, 0.0)
    assert near > far
