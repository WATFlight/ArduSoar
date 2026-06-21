# Sensors & estimation interfaces

This layer sits between the **hardware** and the **guidance brain**. The brain
reads `VehicleState` / `Wind` / `ThermalMap`; it never touches a raw sensor. So
when the real sensors arrive, only the bottom layer changes — guidance is
untouched. You can also tune each sensor's data quality (noise/bias/rate) freely
to see how robust the algorithms are.

## Data flow

```
hardware (or sim)
      │  fills
      ▼
SensorSnapshot          ← raw readings (accel, gyro, GPS, compass, pitot, baro, …)
      │  StateFusion.update()        (proposal 5)
      ▼
VehicleState            ← clean position / velocity / heading / airspeed / vario
      │  WindEstimator.update()      (proposal 4)
      ▼
Wind  ───────────────►  ThermalMap.drift_with_wind()   (proposal 4)
                        ThermalMap.add_or_update()      (proposal 2 + 5 scoring)
                        ThermalMap.best_reachable()     (proposal 2 planning)
```

## Sensor → interface mapping

| Hardware sensor | Reading | Used by |
|---|---|---|
| x,y,z accelerometer | `AccelReading` | fusion (attitude / bank) |
| x,y,z gyroscope | `GyroReading` | fusion (attitude / turn rate) |
| GPS (+ ground speed) | `GPSReading` | fusion (position/velocity), wind, thermal map |
| compass | `CompassReading` | fusion (heading), wind |
| pitot (long/lat airspeed) | `PitotReading` | fusion (airspeed), wind |
| barometer | `BaroReading` | fusion (altitude + **variometer** = lift signal) |
| temp/humidity | `TempHumidityReading` | environment (future: thermal likelihood) |
| camera | `CameraReading` | future (cloud-street / vision cues) |
| radio + video link | — (telemetry, not a sensor) | downlink/uplink, add a `Telemetry` interface when needed |

## Tuning the data (today, in sim)

Everything is configurable in `SensorConfig` (noise std + bias per sensor, GPS
rate). Example: make the variometer noisier and GPS slower —

```python
from sensors.simulated import SensorConfig, NoiseSpec, SimulatedSensorSuite
cfg = SensorConfig()
cfg.baro_vspeed = NoiseSpec(std=1.2)   # noisier vario
cfg.gps_rate_hz = 1.0                  # 1 Hz GPS
suite = SimulatedSensorSuite(cfg)
```

## Swapping in real hardware (later)

1. Subclass `SensorSuite` (e.g. `MavlinkSensorSuite`) whose `read()` pulls from
   MAVLink / I2C / serial instead of `GroundTruth`. Return the same
   `SensorSnapshot`.
2. Replace `PassthroughFusion` with a real EKF/AHRS behind the same
   `StateFusion.update()` signature.
3. Nothing in guidance, the thermal map, or the planners changes.

## Status

These are interface scaffolds with simple working implementations, kept separate
from the existing `run_simulation` so current behaviour and tests are unchanged.
The integration point is the sim loop: build `GroundTruth` via
`ground_truth_from_sim(...)`, read the suite, fuse, then feed the estimator/
controllers from `VehicleState` instead of the raw `GliderState`.
