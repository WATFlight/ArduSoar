"""Tests for the changing-world model and the four lifecycle behaviours."""

import math

from thermal_model.lifecycle_thermal import LifecycleThermal, make_lifecycle_corridor
from thermal_estimator.estimator import ThermalEstimate
from navigation.decision import worth_climbing, net_climb
from navigation.thermal_map import ThermalMap
from navigation.thermal_prior import build_prior, BeliefMap


# --- (1) lifecycle envelope: grow -> hold -> decay -> dead -------------------
def _thermal():
    return LifecycleThermal(0, 0, W0_peak=4.0, R=50, birth=100, t_grow=100, t_hold=200, t_decay=100)


def test_strength_zero_before_birth_and_after_death():
    th = _thermal()
    assert th.strength(50) == 0.0          # not born yet
    assert th.strength(500) == 0.0         # dead (birth+grow+hold+decay = 500)
    assert th.vertical_velocity(0, 0, 50) == 0.0


def test_strength_grows_holds_decays():
    th = _thermal()
    assert abs(th.strength(150) - 2.0) < 1e-6     # halfway up the ramp
    assert abs(th.strength(250) - 4.0) < 1e-6     # plateau
    assert abs(th.strength(450) - 2.0) < 1e-6     # halfway down the decay


def test_dead_thermal_gives_no_lift_at_core():
    th = _thermal()
    assert th.vertical_velocity(0, 0, 600) == 0.0  # core, but dead


# --- (4) value decision: skip weak when high, take anything when low ---------
def _est(W0, R=50):
    return ThermalEstimate(0, 0, W0, R, confidence=1.0)


def test_skip_weak_thermal_when_high():
    # a weak (but still climbing) thermal: not worth it when we're high
    assert net_climb(_est(2.0), 16) > 0.0
    assert worth_climbing(_est(2.0), altitude=800, V=16, low_alt=300, min_climb_comfortable=0.6) is False


def test_take_strong_thermal_when_high():
    assert worth_climbing(_est(4.0), altitude=800, V=16, low_alt=300, min_climb_comfortable=0.6) is True


def test_take_weak_thermal_when_low():
    # same weak thermal, but now we're desperate -> take it (it still climbs)
    assert worth_climbing(_est(2.0), altitude=150, V=16, low_alt=300) is True


# --- (3) online map: add / merge / mark dead --------------------------------
def test_map_marks_dead_thermal_gone():
    m = ThermalMap(merge_dist=80.0)
    m.add_or_update(500, 500, 3.0, 50, t=0.0)
    assert len(m.thermals) == 1
    m.mark_dead(505, 498)                   # flew here, found nothing
    assert len(m.thermals) == 0


# --- (2) belief decay: an aging map is trusted less --------------------------
def test_belief_decay_lowers_unconfirmed_prob():
    belief = BeliefMap(build_prior([(100, 100, 3.0)], (0.0, 0.0), 0.0))
    p0 = belief.candidates[0].prob
    belief.decay(300.0, tau=300.0)
    assert belief.candidates[0].prob < p0 * 0.5   # ~1/e after one tau


# --- world builder produces a mix of mapped + born-later thermals ------------
def test_corridor_has_mapped_and_born_later():
    field, known = make_lifecycle_corridor((0, 0), (5000, 0), 10, seed=1, born_fraction=0.6)
    alive_at_takeoff = [t for t in field.thermals if t.birth <= 0]
    born_later = [t for t in field.thermals if t.birth > 0]
    assert len(known) == len(alive_at_takeoff)     # only alive ones are uploaded
    assert len(born_later) >= 1                     # some appear mid-flight (off-map)
