"""Acceptance tests for goals.md goal 1 (realistic world model)."""

import math

import numpy as np

from thermal_model.random_field import make_random_world, make_uploaded_map


BOUNDS = (0.0, 5000.0, 0.0, 4000.0)


# --- 1.1 random 2-D field (not a corridor) ----------------------------------
def test_thermals_fill_2d_area():
    field = make_random_world(BOUNDS, t_max=2000, seed=1, initial_count=20, spawn_rate=0.0)
    xs = np.array([t.x for t in field.thermals])
    ys = np.array([t.y for t in field.thermals])
    # genuinely spread in BOTH axes (a corridor would collapse one of them)
    assert xs.std() > 800
    assert ys.std() > 600


# --- 1.2 stochastic birth-death (population turns over) ----------------------
def test_live_count_varies_and_new_thermals_appear():
    field = make_random_world(BOUNDS, t_max=2000, seed=2, initial_count=8, spawn_rate=0.02)
    counts = [len(field.alive_thermals(t)) for t in range(0, 2000, 100)]
    assert max(counts) != min(counts)                        # population changes over time
    born_later = [t for t in field.thermals if t.birth > 0]
    assert len(born_later) >= 1                              # thermals not present at takeoff


# --- 1.3 wind drift + meander -----------------------------------------------
def test_thermal_drifts_downwind_but_not_in_a_straight_line():
    field = make_random_world(BOUNDS, t_max=1000, wind=(3.0, -1.0), seed=3, initial_count=1, spawn_rate=0.0)
    th = field.thermals[0]
    c0 = th.center(th.birth)
    c1 = th.center(th.birth + 100.0)
    # net motion is carried downwind by the bulk wind (~ +300, -100)...
    assert (c1[0] - c0[0]) > 200.0
    assert (c1[1] - c0[1]) < -50.0
    # ...but the path is NOT a straight line: the midpoint bends off the chord.
    cm = th.center(th.birth + 50.0)
    chord_mid = ((c0[0] + c1[0]) / 2.0, (c0[1] + c1[1]) / 2.0)
    assert math.hypot(cm[0] - chord_mid[0], cm[1] - chord_mid[1]) > 4.0


# --- 1.4 imperfect uploaded map (global offset) -----------------------------
def test_uploaded_map_is_globally_offset():
    field = make_random_world(BOUNDS, t_max=1000, seed=4, initial_count=15, spawn_rate=0.0)
    offset = (120.0, -80.0)
    uploaded = make_uploaded_map(field, upload_time=0.0, offset=offset, rotation_deg=0.0, pos_noise=0.0)
    alive = [th for th in field.thermals if th.alive(0.0)]
    assert len(uploaded) == len(alive)                      # only alive thermals are seen
    # every uploaded point is shifted by the global offset (no rotation/noise here)
    for (ux, uy, _), th in zip(uploaded, alive):
        tx, ty = th.center(0.0)
        assert abs((ux - tx) - offset[0]) < 1e-6
        assert abs((uy - ty) - offset[1]) < 1e-6


def test_uploaded_map_omits_thermals_born_after_upload():
    field = make_random_world(BOUNDS, t_max=2000, seed=5, initial_count=6, spawn_rate=0.02)
    uploaded = make_uploaded_map(field, upload_time=0.0, pos_noise=0.0)
    # thermals born after upload exist but are not on the map
    assert any(t.birth > 0 for t in field.thermals)
    assert len(uploaded) == len([t for t in field.thermals if t.alive(0.0)])
