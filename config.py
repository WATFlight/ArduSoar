"""Central configuration for the ArduSoar simulation.

All tunable numbers live here so the rest of the code stays readable. Units are
SI (metres, seconds, radians) unless a name says otherwise (``*_deg``).
"""

import math

# --- Physics ---------------------------------------------------------------
G = 9.81  # gravitational acceleration (m/s^2)

# --- Simulation timing -----------------------------------------------------
DT = 0.1          # integration timestep (s)
SIM_TIME = 400.0  # total simulated time (s)

# --- Glider ----------------------------------------------------------------
AIRSPEED = 16.0        # constant airspeed for the basic version (m/s)
BASE_SINK_RATE = 0.7   # sink rate in wings-level flight (m/s)
MAX_BANK_DEG = 45.0    # bank-angle clamp for all controllers (deg)

# Initial glider state
START_X = 0.0
START_Y = 0.0
START_H = 300.0        # starting altitude (m)
START_HEADING = math.radians(45.0)  # pointing toward the thermal / waypoint

# --- True thermal (unknown to the controller) ------------------------------
THERMAL_X = 200.0
THERMAL_Y = 200.0
THERMAL_W0 = 3.5       # peak updraft at the core (m/s)
THERMAL_R = 50.0       # thermal radius / spread (m)

# --- Cruise waypoint (placed beyond the thermal so the path crosses it) ----
WAYPOINT_X = 400.0
WAYPOINT_Y = 400.0

# --- Thermal estimator -----------------------------------------------------
WINDOW_SIZE = 50            # rolling measurement window length
MIN_POINTS_TO_FIT = 25      # need at least this many samples before fitting
DETECT_LIFT_THRESHOLD = 0.6 # min peak measured lift (m/s) to attempt a fit
MIN_THERMAL_STRENGTH = 0.6  # below this fitted W_0 we treat it as "no thermal"
ESTIMATE_EVERY = 5          # run the optimizer every N steps (speed)

# Regularization weights (penalise jumps from the previous estimate)
LAMBDA_W0 = 0.05
LAMBDA_R = 0.01
LAMBDA_POS = 0.02

# --- State machine thresholds ----------------------------------------------
PROBE_THRESHOLD = 0.2     # confidence above this -> at least PROBE
THERMAL_THRESHOLD = 0.5   # confidence above this -> THERMAL

# --- L1 guidance -----------------------------------------------------------
L1_DISTANCE = 30.0  # lookahead distance (m)
