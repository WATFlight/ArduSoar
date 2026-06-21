"""Step-able simulation engine (Phase 0 — the dashboard's backbone).

This mirrors the per-tick logic of ``explore.py`` but exposes it as a stepper so a
UI can drive it one tick at a time, read the live state, and restart it with new
parameters. Everything is parameterised through ``Params`` (no module globals), so
the dashboard's sliders map straight onto fields here.

    p = Params(wind=(0.9, -0.55), airspeed=16.0)
    eng = Engine(p)
    while not eng.done:
        state = eng.step()          # dict: position, mode, wind, battery, thermals, ...
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from config import DT, ESTIMATE_EVERY
from glider_model.glider import GliderState, SimpleGlider
from glider_model.motor import ElectricSustainer
from thermal_model.random_field import make_random_world, make_uploaded_map
from thermal_model.lifecycle_thermal import LifecycleThermal
from thermal_model.merging_field import MergingField
from thermal_estimator.estimator import ThermalEstimator
from controller.state_machine import GuidanceStateMachine, GuidanceMode
from controller.l1_guidance import L1Guidance
from controller.circling_control import CirclingControl
from sensors.simulated import SimulatedSensorSuite, ground_truth_from_sim, SensorConfig, NoiseSpec
from estimation.state_fusion import PassthroughFusion
from estimation.wind_estimator import SimpleWindEstimator
from navigation.thermal_prior import build_prior, BeliefMap
from navigation.decision import worth_climbing
from lifecycle_search import _search_schedule, _expanding_figure8

ORIGIN = (0.0, 0.0)


@dataclass
class Params:
    # world
    seed: int = 4
    wind: tuple = (0.9, -0.55)            # bulk drift (m/s); cores also meander
    bounds: tuple = (-2000.0, 2000.0, -2000.0, 2000.0)
    initial_count: int = 12
    spawn_rate: float = 0.012
    merge_dist: float = 75.0
    # aircraft
    airspeed: float = 16.0
    start_h: float = 740.0
    cloud_base: float = 800.0
    glide_ratio: float = 22.0
    # decision thresholds
    low_alt: float = 320.0
    min_climb: float = 0.3
    map_decay_tau: float = 4000.0
    search_enter_radius: float = 45.0
    # energy / battery
    home_reserve: float = 130.0
    energy_margin: float = 500.0
    return_alt: float = 300.0
    loiter_radius: float = 250.0          # circle this close to home when nothing to climb
    motor_floor: float = 260.0
    motor_ceil: float = 430.0
    battery_wh: float = 40.0
    motor_power_w: float = 600.0          # motor draw when running
    base_power_w: float = 20.0            # avionics draw (always on): FC, Pi, GPS, radio
    motor_climb: float = 1.5
    reserve_wh: float = 8.0               # battery never planned below this (land with reserve)
    # sim
    max_time: float = 6000.0
    sensor_noise: bool = False            # clean sensors by default
    land_radius: float = 120.0
    # optional weather-derived prior: list of (x, y, strength, prob). When set, the
    # glider's map comes from this forecast instead of make_uploaded_map (no cheat).
    external_prior: tuple = None

    @classmethod
    def from_weather(cls, prior, seed=4, **over):
        """Build Params from a weather processor's prior dict (weather/processor.py)."""
        return cls(seed=seed,
                   wind=tuple(prior["wind"]),
                   bounds=tuple(prior["bounds"]),
                   cloud_base=float(prior["cloud_base_m"]),
                   initial_count=int(prior["thermal_count"]),
                   external_prior=[tuple(c) for c in prior["candidates"]],
                   **over)


def _cfg(noise: bool) -> SensorConfig:
    cfg = SensorConfig()
    if not noise:
        for f in ("accel", "gyro", "gps_pos", "gps_vel", "compass", "pitot",
                  "baro_alt", "baro_vspeed", "temp", "humidity"):
            setattr(cfg, f, NoiseSpec(std=0.0))
    return cfg


class Engine:
    def __init__(self, p: Params = None):
        self.p = p or Params()
        self.reset()

    # -- build / restart ----------------------------------------------------
    def reset(self):
        p = self.p
        self._build_field()
        if p.external_prior is not None:
            # forecast map from real weather (an independent guess, not the sim truth)
            sources = p.external_prior
        else:
            sources = make_uploaded_map(self.field, upload_time=0.0, seed=p.seed + 1,
                                        offset=(40.0, -25.0), rotation_deg=1.0, pos_noise=22.0)
        self.belief = BeliefMap(build_prior(sources, (0.0, 0.0), 0.0))
        self.cands = self.belief.candidates

        self.state = GliderState(ORIGIN[0], ORIGIN[1], p.start_h, math.radians(20), p.airspeed)
        self.glider = SimpleGlider(self.state)
        self.suite = SimulatedSensorSuite(_cfg(p.sensor_noise), seed=p.seed)
        self.estimator = ThermalEstimator()
        self.sustainer = ElectricSustainer(p.battery_wh, p.motor_power_w, p.motor_climb,
                                           base_power_w=p.base_power_w)
        self.fusion = PassthroughFusion()
        self.wind_est = SimpleWindEstimator(alpha=0.05)
        self.sm = GuidanceStateMachine(cloud_base=p.cloud_base)
        self.l1 = L1Guidance()
        self.circling = CirclingControl(self.l1)
        self.segs, self.search_total = _search_schedule(p.airspeed)

        self.k = 0
        self.bank = 0.0
        self.estimate = None
        self.target_c = None          # the candidate we're currently heading to
        self.searching = False
        self.search_t = 0.0
        self.prev_mode = None
        self.thermal_entry_h = None
        self.climbs = 0               # thermals worked
        self.motor_time = 0.0
        self.done = False
        self.crashed = False
        self.return_home = False      # latched once battery is only enough to get home
        self.mission_mode = "soaring"  # soaring | return_home | complete | crashed
        self.last = self._snapshot(0.0, 0.0, GuidanceMode.CRUISE)

    def _build_field(self):
        p = self.p
        base = make_random_world(p.bounds, p.max_time, wind=p.wind, seed=p.seed,
                                 initial_count=p.initial_count, spawn_rate=p.spawn_rate,
                                 grow_range=(90, 180), hold_range=(900, 1600), decay_range=(250, 450))
        rng = np.random.default_rng(p.seed + 2)
        twins = []
        for th in rng.choice(base.thermals, size=min(3, len(base.thermals)), replace=False):
            twins.append(LifecycleThermal(
                th.x + float(rng.uniform(-45, 45)), th.y + float(rng.uniform(-45, 45)),
                float(rng.uniform(2.8, 4.0)), float(rng.uniform(45, 60)),
                th.birth + float(rng.uniform(-60, 60)), th.t_grow, th.t_hold, th.t_decay,
                wind=p.wind))
        base.thermals += twins
        self.field = MergingField(base.thermals, merge_dist=p.merge_dist)

    # -- energy helpers (battery-aware fuel gauge) --------------------------
    def _glide_range(self, h, extra=0.0):
        return max(0.0, h - self.p.home_reserve + extra) * self.p.glide_ratio

    def _home_margin(self, extra=0.0):
        s = self.state
        return self._glide_range(s.h, extra) - math.hypot(s.x - ORIGIN[0], s.y - ORIGIN[1])

    # -- return-home energy budget (battery is the binding resource) --------
    def _energy_to_home_wh(self):
        """Battery (Wh) it would take to get home from here: avionics for the whole
        trip, plus motor energy if we cannot glide the distance on altitude alone."""
        s, p = self.state, self.p
        d = math.hypot(s.x - ORIGIN[0], s.y - ORIGIN[1])
        t_home = d / max(s.V, 1.0)                       # straight run home (s)
        motor_time = 0.0
        if self._glide_range(s.h) < d:                   # altitude alone won't reach home
            deficit = (d / p.glide_ratio + p.home_reserve) - s.h   # extra climb needed (m)
            motor_time = max(0.0, deficit) / max(p.motor_climb, 1e-3)
        base_wh = p.base_power_w * (t_home + motor_time) / 3600.0
        motor_wh = p.motor_power_w * motor_time / 3600.0
        return base_wh + motor_wh

    def _spare_energy_wh(self):
        """Battery beyond what's needed to get home and keep the landing reserve.
        <= 0 means: head home now or you won't make it with the reserve intact. The
        1.4x covers the straight-line estimate being optimistic (real path circles /
        motors more on the way)."""
        return self.sustainer.charge_wh - 1.4 * self._energy_to_home_wh() - self.p.reserve_wh

    def _endurance_target(self, extra):
        """Pick the next thermal to work to STAY ALOFT, with the energy monitor:
        before committing we require we can still glide home from it (worst case it
        is dead). When the fuel gauge (height + battery) runs low we prefer a thermal
        CLOSER TO HOME; otherwise the strongest/most-likely one we can reach."""
        s = self.state
        reach = self._glide_range(s.h, extra)
        skip = self.sm._skip_center
        opts = []
        for c in self.belief.active():
            if skip is not None and math.hypot(c.x - skip[0], c.y - skip[1]) < self.sm.skip_radius:
                continue                                  # don't re-target the one we just left
            opts.append(c)
        if not opts:
            return None
        d = lambda c: math.hypot(c.x - s.x, c.y - s.y)
        safe = [c for c in opts if d(c) + math.hypot(c.x, c.y) <= reach]   # reach it AND get home
        pool = safe or [c for c in opts if d(c) <= reach]
        if not pool:
            return None
        if self._spare_energy_wh() < self.p.reserve_wh:         # battery getting low -> stay near home
            return min(pool, key=lambda c: math.hypot(c.x, c.y))
        return max(pool, key=lambda c: c.prob * c.strength_guess)

    def _cand_status(self, c):
        if c.confirmed:
            return "lift"           # climbed a thermal here
        if c.visited:
            return "empty"          # searched, nothing
        if c.prob < self.belief.min_prob:
            return "abandoned"      # map went stale, written off
        return "unsurveyed"

    # -- one tick -----------------------------------------------------------
    def step(self):
        if self.done:
            return self.last
        p = self.p
        s = self.state
        t = self.k * DT

        # battery: avionics always draw; the motor adds a lot when it runs.
        self.sustainer.draw_base(DT)
        in_thermal = self.prev_mode == GuidanceMode.THERMAL
        floor = p.motor_ceil if self.sustainer.on else p.motor_floor
        want_motor = s.h < floor and not in_thermal
        motor_climb = self.sustainer.step(want_motor, DT)
        if self.sustainer.on:
            self.motor_time += DT

        lift = self.field.vertical_velocity(s.x, s.y, t)
        sink = self.glider.sink_rate()
        h_dot = self.glider.step(self.bank, lift + motor_climb, DT)
        if s.h <= 0.0:
            self.done = True
            self.crashed = True
            self.mission_mode = "crashed"
            self.last = self._snapshot(t, h_dot, self.prev_mode or GuidanceMode.CRUISE)
            return self.last

        # vario senses AIR motion only (strip the motor's contribution)
        snap = self.suite.read(ground_truth_from_sim(t, s, h_dot - motor_climb, wind=p.wind))
        vehicle = self.fusion.update(snap)
        wind = self.wind_est.update(vehicle)
        self.estimator.set_wind(wind)
        self.estimator.add_measurement(s.x, s.y, snap.baro.vertical_speed, sink)
        if self.k % ESTIMATE_EVERY == 0:
            self.estimate = self.estimator.estimate()
        self.belief.decay(DT, tau=p.map_decay_tau)
        self.belief.drift(wind, DT)

        # Return-home decision: once the battery is only just enough to get home
        # (plus the landing reserve), commit to going home and don't chase thermals.
        if not self.return_home and self._spare_energy_wh() <= 0.0:
            self.return_home = True
            self.mission_mode = "return_home"

        # While returning home, ignore thermals so the state machine stays in cruise.
        est_sm = None if self.return_home else (self.estimate if (self.estimate is not None
            and worth_climbing(self.estimate, s.h, s.V, low_alt=p.low_alt,
                               min_climb_comfortable=p.min_climb)) else None)
        mode = self.sm.update(est_sm, altitude=s.h, position=(s.x, s.y))
        active = self.sm.active_estimate

        if mode == GuidanceMode.THERMAL and self.prev_mode != GuidanceMode.THERMAL:
            self.thermal_entry_h = s.h
            self.climbs += 1
        if self.prev_mode == GuidanceMode.THERMAL and mode == GuidanceMode.CRUISE:
            climbed = self.thermal_entry_h is not None and (s.h - self.thermal_entry_h) > 50.0
            if self.target_c is not None and climbed:        # worked this one -> hop to others
                self.belief.confirm(self.target_c, self.target_c.x, self.target_c.y,
                                    self.target_c.strength_guess)
            self.target_c = None
            self.searching = False
        self.prev_mode = mode

        # drop a target that has gone stale / been ruled out
        if self.target_c is not None and self.target_c not in self.belief.active():
            self.target_c = None
            self.searching = False

        # --- guidance ---
        if self.return_home:
            # RETURN_HOME: navigate straight to home (motor sustains altitude if low);
            # MISSION_COMPLETE once inside the home radius.
            d_home = math.hypot(s.x - ORIGIN[0], s.y - ORIGIN[1])
            if d_home <= p.land_radius:
                self.done = True
                self.mission_mode = "complete"
            else:
                self.bank = self.l1.bank_to_point(s, *ORIGIN)
        # --- endurance guidance: keep working thermals to stay aloft ---
        elif mode == GuidanceMode.THERMAL and active is not None:
            self.bank = self.circling.command(s, active)
            self.searching = False
        elif self.searching:
            self.search_t += DT
            if self.search_t > self.search_total:            # searched here, no lift
                if self.target_c is not None:
                    self.belief.disconfirm(self.target_c)
                self.target_c = None
                self.searching = False
                self.bank = 0.0
            else:
                self.bank = _expanding_figure8(self.search_t, self.segs)
        else:
            extra = self.sustainer.available_climb_m()
            if self.target_c is None:
                self.target_c = self._endurance_target(extra)
            cand = self.target_c
            if cand is None:
                # nothing reachable to climb -> loiter near home to stay up (don't fly off)
                if math.hypot(s.x, s.y) > p.loiter_radius:
                    self.bank = self.l1.bank_to_point(s, *ORIGIN)
                else:
                    self.bank = math.radians(22.0)
            elif math.hypot(cand.x - s.x, cand.y - s.y) <= p.search_enter_radius:
                self.searching = True
                self.search_t = 0.0
                self.bank = _expanding_figure8(0.0, self.segs)
            else:
                self.bank = self.l1.bank_to_point(s, cand.x, cand.y)

        self.k += 1
        if t >= p.max_time:          # endurance ends only on crash or the time cap
            self.done = True
        self.last = self._snapshot(t, h_dot, mode)
        return self.last

    # -- read-out -----------------------------------------------------------
    def _snapshot(self, t, h_dot, mode):
        s = self.state
        eff = self.field.effective(t)
        thermals = [(cx, cy, W, R, n) for cx, cy, W, R, n in eff if W > 0.1]
        cstat = [(c.x, c.y, self._cand_status(c)) for c in self.cands]
        return {
            "t": t, "x": s.x, "y": s.y, "h": max(s.h, 0.0),
            "mode": mode.value if hasattr(mode, "value") else str(mode),
            "mission": self.mission_mode, "return_home": self.return_home,
            "climb": h_dot, "soc": self.sustainer.soc(), "motor": self.sustainer.on,
            "margin": self._home_margin(self.sustainer.available_climb_m()),
            "spare_wh": self._spare_energy_wh(), "to_home_wh": self._energy_to_home_wh(),
            "d_home": math.hypot(s.x - ORIGIN[0], s.y - ORIGIN[1]),
            "thermals": thermals, "cands": cstat,
            "time_aloft": t, "climbs": self.climbs, "motor_time": self.motor_time,
            "done": self.done, "crashed": self.crashed,
        }


if __name__ == "__main__":
    # quick smoke test
    eng = Engine(Params())
    n = 0
    while not eng.done and n < 70000:
        st = eng.step()
        n += 1
    print(f"endurance: stayed aloft {st['time_aloft']:.0f}s ({st['time_aloft']/60:.1f} min), "
          f"worked {st['climbs']} thermals, motor {st['motor_time']:.0f}s, "
          f"battery {st['soc']*100:.0f}%, crashed={st['crashed']}")
