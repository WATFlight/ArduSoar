"""Sensor abstraction layer.

The guidance/estimation code should read from these interfaces, never from the
raw simulation or raw hardware. Today a *simulated* implementation fills them
from the sim state; tomorrow a *hardware* implementation fills them from the
real sensors (over MAVLink / I2C / serial) — and nothing downstream changes.

Sensors on the aircraft (see sensors/README.md for the mapping):
  accelerometer, gyroscope, GPS, compass, pitot, temp/humidity, barometer,
  camera, radio+video link.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass


# --- ground truth -----------------------------------------------------------
# What a *simulated* sensor corrupts into a reading. A *real* sensor ignores
# this and reads hardware instead. Built from the sim each tick.
@dataclass
class GroundTruth:
    t: float
    # position (local map frame, metres)
    x: float
    y: float
    h: float
    # ground velocity (includes wind), m/s
    vx: float
    vy: float
    vz: float                 # vertical speed = h_dot (the variometer truth)
    heading: float            # rad, CCW from +x
    # body-frame airspeed (no-sideslip in a coordinated turn -> lateral ~ 0)
    airspeed_long: float
    airspeed_lat: float
    # specific force (accelerometer) and angular rate (gyro)
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float
    # environment
    temperature: float
    humidity: float
    pressure: float


# --- per-sensor readings ----------------------------------------------------
@dataclass
class AccelReading:
    ax: float
    ay: float
    az: float


@dataclass
class GyroReading:
    gx: float
    gy: float
    gz: float


@dataclass
class GPSReading:
    x: float
    y: float
    h: float
    vx: float
    vy: float
    ground_speed: float
    track: float              # course over ground, rad


@dataclass
class CompassReading:
    heading: float            # rad


@dataclass
class PitotReading:
    airspeed_longitudinal: float
    airspeed_lateral: float


@dataclass
class BaroReading:
    pressure: float
    altitude: float
    vertical_speed: float     # barometric/TEK variometer


@dataclass
class TempHumidityReading:
    temperature: float
    humidity: float


@dataclass
class CameraReading:
    frame: object             # None in sim; an image/ndarray on hardware


@dataclass
class SensorSnapshot:
    """One synchronized read of the whole suite. Any field may be None if that
    sensor is absent or hasn't produced a sample this tick (e.g. slow GPS)."""

    t: float
    accel: AccelReading | None = None
    gyro: GyroReading | None = None
    gps: GPSReading | None = None
    compass: CompassReading | None = None
    pitot: PitotReading | None = None
    baro: BaroReading | None = None
    temp_humidity: TempHumidityReading | None = None
    camera: CameraReading | None = None


# --- abstract devices -------------------------------------------------------
class Sensor(ABC):
    """One physical device. Simulated subclasses use ``truth``; hardware
    subclasses ignore it and read the bus."""

    @abstractmethod
    def read(self, truth: GroundTruth):
        ...


class SensorSuite(ABC):
    """The full set of sensors, read together into one snapshot."""

    @abstractmethod
    def read(self, truth: GroundTruth) -> SensorSnapshot:
        ...
