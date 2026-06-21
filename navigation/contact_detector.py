"""Thermal contact detection from sensor data.

Decides "am I in a thermal right now?" from the live variometer (reconstructed
lift = vario + sink). Uses a short moving average so a single noisy sample
doesn't trigger it. Extra cues (accelerometer turbulence, temperature rise) can
be folded in here later without changing callers.
"""

from collections import deque


class ContactDetector:
    def __init__(self, lift_threshold: float = 0.8, window: int = 10):
        self.lift_threshold = lift_threshold
        self._buf: deque = deque(maxlen=window)

    def update(self, w_meas: float) -> tuple:
        """Feed reconstructed lift; return (in_contact, smoothed_lift)."""
        self._buf.append(w_meas)
        avg = sum(self._buf) / len(self._buf)
        return avg > self.lift_threshold, avg

    def reset(self) -> None:
        self._buf.clear()
