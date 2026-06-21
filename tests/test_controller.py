"""Tests for the state machine and L1 guidance."""

import math

from glider_model.glider import GliderState
from thermal_estimator.estimator import ThermalEstimate
from controller.state_machine import GuidanceStateMachine, GuidanceMode
from controller.l1_guidance import L1Guidance


# --- state machine ----------------------------------------------------------
def test_no_estimate_is_cruise():
    sm = GuidanceStateMachine()
    assert sm.update(None) == GuidanceMode.CRUISE


def test_low_confidence_is_probe():
    sm = GuidanceStateMachine(probe_threshold=0.2, thermal_threshold=0.5)
    est = ThermalEstimate(0, 0, 3.0, 40.0, confidence=0.3)
    assert sm.update(est) == GuidanceMode.PROBE


def test_high_confidence_is_thermal():
    sm = GuidanceStateMachine(probe_threshold=0.2, thermal_threshold=0.5)
    est = ThermalEstimate(0, 0, 3.0, 40.0, confidence=0.8)
    assert sm.update(est) == GuidanceMode.THERMAL


# --- L1 guidance ------------------------------------------------------------
def _state_heading_east():
    # Flying along +x; "left" is +y, "right" is -y.
    return GliderState(0.0, 0.0, 300.0, 0.0, 16.0, 0.0)


def test_target_left_banks_left():
    l1 = L1Guidance()
    phi = l1.bank_to_point(_state_heading_east(), target_x=10.0, target_y=10.0)
    assert phi > 0.0  # positive bank == left turn


def test_target_right_banks_right():
    l1 = L1Guidance()
    phi = l1.bank_to_point(_state_heading_east(), target_x=10.0, target_y=-10.0)
    assert phi < 0.0


def test_bank_is_clamped():
    l1 = L1Guidance(L1=5.0, max_bank_deg=45.0)
    # Target hard to the left should saturate the clamp.
    phi = l1.bank_to_point(_state_heading_east(), target_x=0.0, target_y=50.0)
    assert phi <= math.radians(45.0) + 1e-9
    assert phi >= math.radians(44.0)  # effectively at the clamp
