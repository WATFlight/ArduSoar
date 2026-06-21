"""Simulated sensors: fill the interfaces from sim ground truth + configurable
noise/bias, so you can tune each sensor's data quality without touching the
guidance code. Swap these for hardware implementations later — same interfaces.

Tune data quality via ``SensorConfig`` (noise std + bias per sensor, plus GPS
update rate). Pass a custom config to ``SimulatedSensorSuite`` to experiment.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from config import G
from glider_model.glider import GliderState
from sensors.interfaces import (
    GroundTruth, SensorSnapshot, SensorSuite,
    AccelReading, GyroReading, GPSReading, CompassReading,
    PitotReading, BaroReading, TempHumidityReading, CameraReading,
)

_P0 = 101325.0  # sea-level pressure (Pa)


def pressure_at(h: float) -> float:
    """ISA barometric pressure (Pa) at altitude h (m)."""
    return _P0 * (1.0 - 2.25577e-5 * h) ** 5.25588


# --- build ground truth from the kinematic sim ------------------------------
def ground_truth_from_sim(
    t: float,
    state: GliderState,
    h_dot: float,
    wind: tuple = (0.0, 0.0),
    temperature: float = 15.0,
    humidity: float = 50.0,
) -> GroundTruth:
    """Derive the perfect values each sensor would measure, from the sim state.

    Coordinated-turn assumptions: no sideslip (lateral airspeed ~ 0), turn rate
    gz = g*tan(phi)/V, load factor n = 1/cos(phi) along body-z.
    """
    V, hdg, phi = state.V, state.heading, state.bank_angle
    vx = V * math.cos(hdg) + wind[0]   # ground velocity = air velocity + wind
    vy = V * math.sin(hdg) + wind[1]
    n = 1.0 / max(math.cos(phi), 1e-3)
    return GroundTruth(
        t=t, x=state.x, y=state.y, h=state.h,
        vx=vx, vy=vy, vz=h_dot, heading=hdg,
        airspeed_long=V, airspeed_lat=0.0,
        ax=0.0, ay=G * math.tan(phi), az=G * n,   # specific force in a coordinated turn
        gx=0.0, gy=0.0, gz=G * math.tan(phi) / V,
        temperature=temperature, humidity=humidity, pressure=pressure_at(state.h),
    )


# --- noise model + per-sensor config ----------------------------------------
@dataclass
class NoiseSpec:
    std: float = 0.0    # Gaussian noise standard deviation
    bias: float = 0.0   # constant offset


@dataclass
class SensorConfig:
    """Edit these to change how good/bad each sensor's data is."""
    accel: NoiseSpec = field(default_factory=lambda: NoiseSpec(std=0.15))      # m/s^2
    gyro: NoiseSpec = field(default_factory=lambda: NoiseSpec(std=0.01))       # rad/s
    gps_pos: NoiseSpec = field(default_factory=lambda: NoiseSpec(std=1.5))     # m
    gps_vel: NoiseSpec = field(default_factory=lambda: NoiseSpec(std=0.2))     # m/s
    compass: NoiseSpec = field(default_factory=lambda: NoiseSpec(std=math.radians(2.0)))
    pitot: NoiseSpec = field(default_factory=lambda: NoiseSpec(std=0.3))       # m/s
    baro_alt: NoiseSpec = field(default_factory=lambda: NoiseSpec(std=0.5))    # m
    baro_vspeed: NoiseSpec = field(default_factory=lambda: NoiseSpec(std=0.3)) # m/s (vario)
    temp: NoiseSpec = field(default_factory=lambda: NoiseSpec(std=0.2))
    humidity: NoiseSpec = field(default_factory=lambda: NoiseSpec(std=1.0))
    gps_rate_hz: float = 5.0    # GPS updates slower than the loop
    has_camera: bool = False


class SimulatedSensorSuite(SensorSuite):
    def __init__(self, config: SensorConfig = None, seed: int = 0):
        self.cfg = config or SensorConfig()
        self._rng = np.random.default_rng(seed)
        self._last_gps_t = -1e9
        self._last_gps: GPSReading | None = None

    def _n(self, spec: NoiseSpec) -> float:
        return spec.bias + (self._rng.normal(0.0, spec.std) if spec.std > 0 else 0.0)

    def read(self, truth: GroundTruth) -> SensorSnapshot:
        c = self.cfg
        accel = AccelReading(truth.ax + self._n(c.accel), truth.ay + self._n(c.accel), truth.az + self._n(c.accel))
        gyro = GyroReading(truth.gx + self._n(c.gyro), truth.gy + self._n(c.gyro), truth.gz + self._n(c.gyro))
        compass = CompassReading(truth.heading + self._n(c.compass))
        pitot = PitotReading(truth.airspeed_long + self._n(c.pitot), truth.airspeed_lat + self._n(c.pitot))
        alt = truth.h + self._n(c.baro_alt)
        baro = BaroReading(pressure_at(truth.h) + self._n(NoiseSpec()), alt, truth.vz + self._n(c.baro_vspeed))
        th = TempHumidityReading(truth.temperature + self._n(c.temp), truth.humidity + self._n(c.humidity))

        # GPS runs slower than the control loop.
        if truth.t - self._last_gps_t >= 1.0 / c.gps_rate_hz - 1e-9:
            vx = truth.vx + self._n(c.gps_vel)
            vy = truth.vy + self._n(c.gps_vel)
            self._last_gps = GPSReading(
                x=truth.x + self._n(c.gps_pos), y=truth.y + self._n(c.gps_pos), h=truth.h + self._n(c.gps_pos),
                vx=vx, vy=vy, ground_speed=math.hypot(vx, vy), track=math.atan2(vy, vx),
            )
            self._last_gps_t = truth.t

        camera = CameraReading(frame=None) if c.has_camera else None
        return SensorSnapshot(
            t=truth.t, accel=accel, gyro=gyro, gps=self._last_gps, compass=compass,
            pitot=pitot, baro=baro, temp_humidity=th, camera=camera,
        )
