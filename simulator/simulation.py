"""Simulation loop (Step 10 of the proposal).

Wires the true thermal, the glider, the estimator, the state machine, and the
three controllers together, logging everything for later plotting.
"""

from dataclasses import dataclass, field

import numpy as np

from config import (
    DT,
    SIM_TIME,
    AIRSPEED,
    START_X,
    START_Y,
    START_H,
    START_HEADING,
    THERMAL_X,
    THERMAL_Y,
    THERMAL_W0,
    THERMAL_R,
    WAYPOINT_X,
    WAYPOINT_Y,
    ESTIMATE_EVERY,
)
from glider_model.glider import GliderState, SimpleGlider
from thermal_model.thermal import GaussianThermal
from thermal_model.thermal_field import ThermalField
from thermal_estimator.estimator import ThermalEstimator
from controller.state_machine import GuidanceStateMachine, GuidanceMode
from controller.l1_guidance import L1Guidance
from controller.cruise_control import CruiseControl
from controller.probe_control import ProbeControl
from controller.circling_control import CirclingControl


@dataclass
class SimLog:
    """Column-oriented log; each list grows by one entry per timestep."""

    t: list = field(default_factory=list)
    x: list = field(default_factory=list)
    y: list = field(default_factory=list)
    h: list = field(default_factory=list)
    heading: list = field(default_factory=list)
    bank_angle: list = field(default_factory=list)
    h_dot: list = field(default_factory=list)
    thermal_lift: list = field(default_factory=list)
    sink_rate: list = field(default_factory=list)
    mode: list = field(default_factory=list)
    est_x_c: list = field(default_factory=list)
    est_y_c: list = field(default_factory=list)
    est_W_0: list = field(default_factory=list)
    est_R_th: list = field(default_factory=list)
    confidence: list = field(default_factory=list)


def run_simulation(
    optimize_circling: bool = False,
    thermal_x: float = THERMAL_X,
    thermal_y: float = THERMAL_Y,
    thermal_w0: float = THERMAL_W0,
    thermal_r: float = THERMAL_R,
    sim_time: float = SIM_TIME,
    detect_threshold: float = None,
    probe_threshold: float = None,
    vario_noise_std: float = 0.0,
    noise_seed: int = 0,
    cloud_base: float = None,
    waypoint_x: float = WAYPOINT_X,
    waypoint_y: float = WAYPOINT_Y,
    field: ThermalField = None,
    start_h: float = START_H,
) -> SimLog:
    # Variometer measurement noise (only corrupts what the estimator sees; the
    # true altitude integration stays clean). Off by default.
    rng = np.random.default_rng(noise_seed)

    # --- world + glider ---
    # A single thermal (default) is just a one-element field, so the loop below
    # is identical for one or many thermals.
    if field is None:
        field = ThermalField([GaussianThermal(thermal_x, thermal_y, thermal_w0, thermal_r)])
    state = GliderState(START_X, START_Y, start_h, START_HEADING, AIRSPEED)
    glider = SimpleGlider(state)

    # --- estimation + guidance ---
    estimator = (
        ThermalEstimator()
        if detect_threshold is None
        else ThermalEstimator(detect_threshold=detect_threshold)
    )
    sm_kwargs = {}
    if probe_threshold is not None:
        sm_kwargs["probe_threshold"] = probe_threshold
    if cloud_base is not None:
        sm_kwargs["cloud_base"] = cloud_base
    state_machine = GuidanceStateMachine(**sm_kwargs)
    l1 = L1Guidance()
    cruise = CruiseControl(waypoint_x, waypoint_y, l1)
    probe = ProbeControl()
    circling = CirclingControl(l1, optimize=optimize_circling)

    log = SimLog()
    bank_command = 0.0
    estimate = None
    prev_mode = None

    n_steps = int(sim_time / DT)
    for k in range(n_steps):
        t = k * DT

        # 1-2. True thermal lift (summed over the field) and sink rate.
        lift = field.vertical_velocity(state.x, state.y)
        sink = glider.sink_rate()

        # 3-5. Step the glider with the previously chosen command; get net climb.
        h_dot = glider.step(bank_command, lift, DT)

        # 6. Feed the measurement to the estimator (with optional vario noise).
        h_dot_meas = h_dot + (rng.normal(0.0, vario_noise_std) if vario_noise_std > 0 else 0.0)
        estimator.add_measurement(state.x, state.y, h_dot_meas, sink)

        # 7. Re-run the estimator periodically (cheaper than every step).
        if k % ESTIMATE_EVERY == 0:
            estimate = estimator.estimate()

        # 8. Pick the flight mode (the machine may retain a latched estimate).
        mode = state_machine.update(estimate, altitude=state.h, position=(state.x, state.y))
        active = state_machine.active_estimate

        # On leaving a thermal, restart the cruise schedule (glide on course
        # before searching again).
        if prev_mode == GuidanceMode.THERMAL and mode == GuidanceMode.CRUISE:
            cruise.reset()
        prev_mode = mode

        # 9. Choose the next bank command for the selected mode.
        if mode == GuidanceMode.THERMAL and active is not None:
            bank_command = circling.command(state, active)
        elif mode == GuidanceMode.PROBE and active is not None:
            bank_command = probe.command(state, active)
        else:
            bank_command = cruise.command(state)

        # 10. Log everything.
        log.t.append(t)
        log.x.append(state.x)
        log.y.append(state.y)
        log.h.append(state.h)
        log.heading.append(state.heading)
        log.bank_angle.append(state.bank_angle)
        log.h_dot.append(h_dot)
        log.thermal_lift.append(lift)
        log.sink_rate.append(sink)
        log.mode.append(mode.value)
        log.est_x_c.append(active.x_c if active else None)
        log.est_y_c.append(active.y_c if active else None)
        log.est_W_0.append(active.W_0 if active else None)
        log.est_R_th.append(active.R_th if active else None)
        log.confidence.append(active.confidence if active else 0.0)

    return log
