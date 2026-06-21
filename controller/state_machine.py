"""Guidance state machine (Step 5 of the proposal, + capture & cross-country).

Picks the flight mode from the current thermal estimate and its confidence.

Two behaviours on top of the basic cruise/probe/thermal logic:

* Capture hysteresis: once committed to THERMAL, keep circling the last good
  estimate through brief estimate dropouts (``hold_time``) so the circling law
  can spiral back into a thermal the glider only grazed.

* Cloud-base departure (cross-country): when the glider reaches ``cloud_base``
  it leaves the thermal and remembers where it was (``_skip_center``). While that
  memory is live it refuses to re-enter a thermal near that spot, but is free to
  climb a *different* thermal further along. The memory clears once the glider
  has flown ``skip_clear_dist`` away, so the field can be used as a sequence of
  climbs (thermal hopping) instead of latching onto the first one forever.

``active_estimate`` exposes the estimate the controllers should actually use
(the live one, or the retained one while latched).
"""

from __future__ import annotations

import math
from enum import Enum

from thermal_estimator.estimator import ThermalEstimate
from config import PROBE_THRESHOLD, THERMAL_THRESHOLD, DT


class GuidanceMode(Enum):
    CRUISE = "cruise"
    PROBE = "probe"
    THERMAL = "thermal"


class GuidanceStateMachine:
    def __init__(
        self,
        probe_threshold: float = PROBE_THRESHOLD,
        thermal_threshold: float = THERMAL_THRESHOLD,
        hold_time: float = 10.0,
        dt: float = DT,
        cloud_base: float = None,
        skip_radius: float = 100.0,
        skip_clear_dist: float = 150.0,
    ):
        self.probe_threshold = probe_threshold
        self.thermal_threshold = thermal_threshold
        self.hold_steps = int(hold_time / dt)
        self.cloud_base = cloud_base            # leave a thermal at this altitude
        self.skip_radius = skip_radius          # estimates this close to a left thermal are ignored
        self.skip_clear_dist = skip_clear_dist  # fly this far to forget a left thermal
        self.mode = GuidanceMode.CRUISE
        self.active_estimate: ThermalEstimate | None = None
        self._last_good: ThermalEstimate | None = None
        self._lost = 0
        self._skip_center: tuple | None = None

    def update(
        self,
        estimate: ThermalEstimate | None,
        altitude: float = None,
        position: tuple = None,
    ) -> GuidanceMode:
        # --- cloud-base departure bookkeeping ---
        if self.cloud_base is not None and altitude is not None:
            if (
                self.mode == GuidanceMode.THERMAL
                and altitude >= self.cloud_base
                and self.active_estimate is not None
            ):
                # Reached the ceiling: remember the thermal we're leaving.
                self._skip_center = (self.active_estimate.x_c, self.active_estimate.y_c)
            if self._skip_center is not None and position is not None:
                if math.hypot(position[0] - self._skip_center[0], position[1] - self._skip_center[1]) > self.skip_clear_dist:
                    self._skip_center = None  # flown clear; allow climbing again

        suppressed = self._is_suppressed(estimate)

        # --- mode selection ---
        if self.mode == GuidanceMode.THERMAL:
            if estimate is not None and not suppressed:
                # keep circling the live estimate
                self._commit(estimate)
                self.mode = GuidanceMode.THERMAL
            elif (
                estimate is None
                and self._skip_center is None
                and self._lost < self.hold_steps
                and self._last_good is not None
            ):
                # tolerate a brief dropout: keep circling the retained estimate
                self._lost += 1
                self.active_estimate = self._last_good
                self.mode = GuidanceMode.THERMAL
            else:
                # lost it, or left it at cloud base -> back to cruise
                self._release()
                self.mode = GuidanceMode.CRUISE
            return self.mode

        # Not committed yet (CRUISE / PROBE).
        if estimate is None or suppressed:
            self.active_estimate = None
            self.mode = GuidanceMode.CRUISE
        else:
            self._commit(estimate)
            if estimate.confidence >= self.thermal_threshold:
                self.mode = GuidanceMode.THERMAL
            elif estimate.confidence >= self.probe_threshold:
                self.mode = GuidanceMode.PROBE
            else:
                self.mode = GuidanceMode.CRUISE
        return self.mode

    # -- helpers ------------------------------------------------------------
    def _is_suppressed(self, estimate: ThermalEstimate | None) -> bool:
        """True if this estimate is the thermal we just left at cloud base."""
        if self._skip_center is None or estimate is None:
            return False
        return math.hypot(estimate.x_c - self._skip_center[0], estimate.y_c - self._skip_center[1]) < self.skip_radius

    def _commit(self, estimate: ThermalEstimate) -> None:
        self._last_good = estimate
        self._lost = 0
        self.active_estimate = estimate

    def _release(self) -> None:
        self.active_estimate = None
        self._last_good = None
        self._lost = 0
