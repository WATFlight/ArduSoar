"""Online thermal estimator (Steps 3 & 4 of the proposal).

It keeps a rolling window of ``(x, y, w_meas)`` samples and fits a Gaussian
thermal to them with regularized nonlinear least squares. ``w_meas`` is the
thermal lift reconstructed from the variometer:

    w_meas = h_dot + sink_rate
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares

from config import (
    DT,
    WINDOW_SIZE,
    MIN_POINTS_TO_FIT,
    DETECT_LIFT_THRESHOLD,
    MIN_THERMAL_STRENGTH,
    LAMBDA_W0,
    LAMBDA_R,
    LAMBDA_POS,
)

# If a stored estimate is farther than this from the current window, treat it as
# stale (the glider has moved on to a different thermal).
STALE_PREV_DIST = 200.0


@dataclass
class ThermalEstimate:
    x_c: float
    y_c: float
    W_0: float
    R_th: float
    confidence: float


class ThermalEstimator:
    def __init__(
        self,
        window_size: int = WINDOW_SIZE,
        detect_threshold: float = DETECT_LIFT_THRESHOLD,
        min_strength: float = MIN_THERMAL_STRENGTH,
    ):
        self.window_size = window_size
        self.detect_threshold = detect_threshold
        self.min_strength = min_strength
        # Each entry is (x, y, w_meas). deque drops old samples automatically.
        self._window: deque = deque(maxlen=window_size)
        self._prev: ThermalEstimate | None = None
        self._prev_t = 0.0
        self.wind = (0.0, 0.0)  # known/estimated wind; thermals drift with it
        self._t = 0.0

    def set_wind(self, wind) -> None:
        """Tell the estimator the wind so it can fit the thermal in the frame
        that moves with it (a drifting thermal is static in that frame)."""
        self.wind = (wind.wx, wind.wy) if hasattr(wind, "wx") else (wind[0], wind[1])

    # -- measurement collection (Step 3) ------------------------------------
    def add_measurement(self, x: float, y: float, h_dot: float, sink_rate: float) -> None:
        """Store the sample (absolute position + timestamp)."""
        self._t += DT
        w_meas = h_dot + sink_rate
        self._window.append((x, y, w_meas, self._t))

    # -- fitting (Step 4) ---------------------------------------------------
    def estimate(self) -> ThermalEstimate | None:
        """Fit the Gaussian thermal to the current window.

        Returns ``None`` when there is too little data, no real lift signal, or
        the fit collapses to a negligible thermal.
        """
        if len(self._window) < MIN_POINTS_TO_FIT:
            return None

        data = np.array(self._window)  # shape (N, 4): x, y, w, t
        xs_abs, ys_abs, ws, ts = data[:, 0], data[:, 1], data[:, 2], data[:, 3]

        # No meaningful lift in the window -> there is nothing to track.
        if ws.max() < self.detect_threshold:
            return None

        # De-drift WITHIN the window only: advect each sample to the latest
        # sample's time (gap <= window span, a few seconds), so a drifting
        # thermal looks static for the fit. Using absolute time would multiply
        # any wind error by the whole flight time and blow the fit up.
        t_now = float(ts.max())
        wx, wy = self.wind
        xs = xs_abs + wx * (t_now - ts)
        ys = ys_abs + wy * (t_now - ts)

        # Advect the previous estimate forward to now, so regularization expects
        # the thermal to have drifted instead of fighting its motion. Build a
        # fresh ThermalEstimate rather than mutating in place: the object we
        # returned last call may still be held by the guidance state machine, and
        # silently shifting it under that reference would be a surprising alias.
        if self._prev is not None:
            dt_adv = t_now - self._prev_t
            self._prev = ThermalEstimate(
                self._prev.x_c + wx * dt_adv, self._prev.y_c + wy * dt_adv,
                self._prev.W_0, self._prev.R_th, self._prev.confidence,
            )
        self._prev_t = t_now

        # Drop a stale previous estimate: if the glider has moved to a new area
        # (e.g. hopped to the next thermal in a cross-country), the old estimate
        # would be an infeasible initial guess and its regularization would drag
        # the new fit backward. Fit the local thermal fresh instead.
        if self._prev is not None:
            cx, cy = float(xs.mean()), float(ys.mean())
            if np.hypot(self._prev.x_c - cx, self._prev.y_c - cy) > STALE_PREV_DIST:
                self._prev = None

        p0 = self._initial_guess(xs, ys, ws)
        bounds = self._bounds(xs, ys)
        p0 = np.clip(p0, bounds[0], bounds[1])  # keep the guess strictly feasible

        result = least_squares(
            self._residuals,
            p0,
            bounds=bounds,
            args=(xs, ys, ws),
            method="trf",
            max_nfev=100,
        )
        x_c, y_c, W_0, R_th = result.x

        # A fit that collapses to ~0 strength means "no thermal here".
        if W_0 < self.min_strength:
            self._prev = None
            return None

        confidence = self._confidence(x_c, y_c, W_0, R_th, xs, ys, ws)
        # the fit is already at the current time (we advected the window to t_now)
        est = ThermalEstimate(float(x_c), float(y_c), float(W_0), float(R_th), confidence)
        self._prev = est
        return est

    # -- helpers ------------------------------------------------------------
    def _model(self, params, xs, ys):
        x_c, y_c, W_0, R_th = params
        r2 = (xs - x_c) ** 2 + (ys - y_c) ** 2
        return W_0 * np.exp(-r2 / (R_th ** 2))

    def _residuals(self, params, xs, ys, ws):
        """Data residuals plus regularization toward the previous estimate."""
        res = self._model(params, xs, ys) - ws
        if self._prev is not None:
            x_c, y_c, W_0, R_th = params
            p = self._prev
            reg = [
                np.sqrt(LAMBDA_W0) * (W_0 - p.W_0),
                np.sqrt(LAMBDA_R) * (R_th - p.R_th),
                np.sqrt(LAMBDA_POS) * (x_c - p.x_c),
                np.sqrt(LAMBDA_POS) * (y_c - p.y_c),
            ]
            res = np.concatenate([res, reg])
        return res

    def _initial_guess(self, xs, ys, ws):
        if self._prev is not None:
            p = self._prev
            return [p.x_c, p.y_c, p.W_0, p.R_th]
        # Lift-weighted centroid is a good first guess for the core position.
        weights = np.clip(ws, 0.0, None)
        total = weights.sum()
        if total > 1e-6:
            x_c0 = float((weights * xs).sum() / total)
            y_c0 = float((weights * ys).sum() / total)
        else:
            x_c0, y_c0 = float(xs.mean()), float(ys.mean())
        return [x_c0, y_c0, float(max(ws.max(), 0.5)), 40.0]

    def _bounds(self, xs, ys):
        # Keep the centre near the data so the optimiser cannot wander off.
        margin = 300.0
        lo = [xs.min() - margin, ys.min() - margin, 0.1, 1.0]
        hi = [xs.max() + margin, ys.max() + margin, 10.0, 200.0]
        return (lo, hi)

    def _confidence(self, x_c, y_c, W_0, R_th, xs, ys, ws):
        """Option A from the proposal: confidence = 1 / (1 + mse)."""
        pred = self._model((x_c, y_c, W_0, R_th), xs, ys)
        mse = float(np.mean((pred - ws) ** 2))
        return 1.0 / (1.0 + mse)
