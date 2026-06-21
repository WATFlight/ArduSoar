"""Value-based commit decision (MacCready-flavoured).

Because thermals strengthen and weaken, the choice to circle a given thermal
should depend on its *current* estimated net climb and how badly the glider
needs altitude:

  * high and comfortable  -> only circle a genuinely good thermal,
  * low and in trouble    -> take any lift that keeps you up.

This lets the glider skip a dying/weak thermal when it can afford to, and grab
anything when it can't.
"""

import math

from config import G, BASE_SINK_RATE


def net_climb(estimate, V: float, base_sink: float = BASE_SINK_RATE) -> float:
    """Best steady net climb this thermal can give at speed V (circle at ~0.5R)."""
    R = max(0.5 * estimate.R_th, 10.0)
    w = estimate.W_0 * math.exp(-(R ** 2) / (estimate.R_th ** 2))
    phi = math.atan(V ** 2 / (G * R))
    sink = base_sink / max(math.cos(phi), 1e-3)
    return w - sink


def worth_climbing(estimate, altitude: float, V: float,
                   low_alt: float = 250.0,
                   min_climb_comfortable: float = 0.6) -> bool:
    """Decide whether to commit to circling this thermal.

    Below ``low_alt`` take anything that climbs at all (survival); otherwise only
    commit if the net climb clears ``min_climb_comfortable``."""
    nc = net_climb(estimate, V)
    if altitude < low_alt:
        return nc > 0.0
    return nc > min_climb_comfortable
