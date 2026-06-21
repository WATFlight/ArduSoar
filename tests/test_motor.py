"""Electric sustainer motor + battery (option A)."""

import math
from dataclasses import dataclass

import explore
from glider_model.motor import ElectricSustainer


def test_full_battery_and_available_climb():
    m = ElectricSustainer(capacity_wh=40.0, power_w=600.0, climb_rate=1.5)
    assert m.soc() == 1.0
    # 40 Wh at 600 W / 1.5 m/s climb -> 40 / ((600/1.5)/3600) = 360 m of climb
    assert abs(m.available_climb_m() - 360.0) < 1e-6


def test_motor_drains_only_when_on():
    m = ElectricSustainer(capacity_wh=40.0, power_w=600.0, climb_rate=1.5)
    assert m.step(want_on=False, dt=1.0) == 0.0 and not m.on and m.soc() == 1.0
    climb = m.step(want_on=True, dt=1.0)
    assert climb == 1.5 and m.on and m.soc() < 1.0          # produced climb, drew charge


def test_battery_depletes_to_empty():
    m = ElectricSustainer(capacity_wh=40.0, power_w=600.0, climb_rate=1.5)
    # 40 Wh / 600 W = 240 s of running; run well past that so it clamps to empty
    for _ in range(3000):
        m.step(want_on=True, dt=0.1)
    assert m.soc() == 0.0
    assert m.step(want_on=True, dt=0.1) == 0.0              # empty -> no more climb
    assert m.available_climb_m() == 0.0


@dataclass
class _State:
    x: float
    y: float
    h: float


def test_battery_extends_home_reach():
    """The battery's available climb buys extra range in the fuel gauge."""
    st = _State(2500.0, 0.0, 250.0)
    glide_only = explore._home_margin(st, extra_climb=0.0)
    with_motor = explore._home_margin(st, extra_climb=360.0)
    assert with_motor > glide_only
    # each climb metre buys GLIDE_RATIO metres of glide
    assert abs((with_motor - glide_only) - 360.0 * explore.GLIDE_RATIO) < 1e-6
