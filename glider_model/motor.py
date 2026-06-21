"""Electric sustainer motor + battery (option A: a real, depletable 电量).

Many self-launching / sustainer sailplanes carry a small electric motor and a
battery. The motor can hold height or climb when there is no lift, at the cost of
draining the battery. Unlike altitude (which thermals refill), the battery only
goes DOWN — so it is a genuine consumable the glider must budget.

Model: while the motor is on it adds a fixed net climb rate and burns power.
  - ``soc()``               state of charge, 0..1
  - ``available_climb_m()`` how many more metres of climb the charge can buy
  - ``step(want_on, dt)``   drain (if on) and return the climb contribution (m/s)
"""

from __future__ import annotations


class ElectricSustainer:
    def __init__(self, capacity_wh: float = 40.0, power_w: float = 600.0,
                 climb_rate: float = 1.5, base_power_w: float = 0.0):
        self.capacity_wh = capacity_wh
        self.charge_wh = capacity_wh
        self.power_w = power_w          # motor draw while running
        self.base_power_w = base_power_w  # avionics: FC, Pi, GPS, radio, sensors
        self.climb_rate = climb_rate    # net climb the motor adds (m/s)
        self.on = False

    def draw_base(self, dt: float) -> None:
        """Always-on avionics draw, independent of the motor. The battery goes
        down even when gliding, so it is a continuous resource to budget."""
        self.charge_wh = max(0.0, self.charge_wh - self.base_power_w * dt / 3600.0)

    def soc(self) -> float:
        return self.charge_wh / self.capacity_wh if self.capacity_wh > 0 else 0.0

    def _wh_per_metre(self) -> float:
        # energy to climb 1 m at climb_rate: power * (1/climb_rate) seconds
        return (self.power_w / self.climb_rate) / 3600.0

    def available_climb_m(self) -> float:
        """Metres of climb the remaining charge can still provide."""
        wpm = self._wh_per_metre()
        return self.charge_wh / wpm if wpm > 0 else 0.0

    def step(self, want_on: bool, dt: float) -> float:
        """Run the motor for this step if asked and there is charge left.
        Returns the climb contribution (m/s) to add to the glider's lift."""
        if want_on and self.charge_wh > 0.0:
            self.on = True
            self.charge_wh = max(0.0, self.charge_wh - self.power_w * dt / 3600.0)
            return self.climb_rate if self.charge_wh > 0.0 else 0.0
        self.on = False
        return 0.0
