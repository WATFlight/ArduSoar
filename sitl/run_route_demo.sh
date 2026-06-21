#!/usr/bin/env bash
# Validate the ground planner end-to-end: generate a SITL-local route with the
# planner, then fly that exact .waypoints mission in SITL with ArduSoar on.
set -uo pipefail

VENV=/Users/momo/Downloads/dd/soar-venv/bin/python
SYS=python3
ARDUPILOT=/Users/momo/Downloads/dd/ardupilot
REPO="$(cd "$(dirname "$0")/.." && pwd)"

pkill -9 -f "sim_vehicle.py" 2>/dev/null
pkill -9 -f "build/sitl/bin/arduplane" 2>/dev/null
pkill -9 -f run_in_terminal_window 2>/dev/null
sleep 2

cd "$REPO"
# SITL-local prior (strong candidate at the SITL thermal), then plan a route with
# a low ceiling so the climb stays inside SITL's soaring band (SOAR_ALT_MAX=350).
$VENV companion/make_sitl_prior.py
$SYS -m planner.route_planner --prior companion/sitl_prior.json \
     --takeoff-alt 120 --ceiling-alt 300 2>&1 | grep -vE "NotOpenSSL|warnings.warn"
ROUTE="$(ls -t "$REPO"/planner/routes/route_*.waypoints | head -1)"
echo "[route-demo] flying: $ROUTE"

cd "$ARDUPILOT"
$VENV Tools/autotest/sim_vehicle.py -v ArduPlane -f plane-soaring \
    --no-mavproxy --no-rebuild -w --speedup 20 > /tmp/sitl_route.log 2>&1 &
for i in $(seq 1 60); do
    lsof -nP -iTCP:5760 -sTCP:LISTEN >/dev/null 2>&1 && break
    sleep 1
done
sleep 5

cd "$REPO"
$VENV sitl/fly_route.py --conn tcp:127.0.0.1:5760 --route "$ROUTE" --timeout 300
RC=$?

pkill -9 -f "sim_vehicle.py" 2>/dev/null
pkill -9 -f "build/sitl/bin/arduplane" 2>/dev/null
pkill -9 -f run_in_terminal_window 2>/dev/null
echo "[route-demo] exit $RC"
exit $RC
