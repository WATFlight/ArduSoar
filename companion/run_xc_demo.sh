#!/usr/bin/env bash
# One-shot: fresh plane-soaring SITL, run the weather-guided cross-country relay
# (hop between forecast hotspots), then tear down.
set -uo pipefail

VENV=/Users/momo/Downloads/dd/soar-venv/bin/python
ARDUPILOT=/Users/momo/Downloads/dd/ardupilot
REPO="$(cd "$(dirname "$0")/.." && pwd)"
HERE="$REPO/companion"

pkill -9 -f "sim_vehicle.py" 2>/dev/null
pkill -9 -f "build/sitl/bin/arduplane" 2>/dev/null
pkill -9 -f run_in_terminal_window 2>/dev/null
sleep 2

$VENV "$HERE/make_sitl_prior.py"

cd "$ARDUPILOT"
$VENV Tools/autotest/sim_vehicle.py -v ArduPlane -f plane-soaring \
    --no-mavproxy --no-rebuild -w --speedup 20 > /tmp/sitl_xc.log 2>&1 &
for i in $(seq 1 60); do
    lsof -nP -iTCP:5760 -sTCP:LISTEN >/dev/null 2>&1 && break
    sleep 1
done
sleep 5

cd "$REPO"
$VENV "$HERE/weather_guided_xc.py" --conn tcp:127.0.0.1:5760 \
    --prior "$HERE/sitl_prior.json" --origin home --max-hops 3
RC=$?

pkill -9 -f "sim_vehicle.py" 2>/dev/null
pkill -9 -f "build/sitl/bin/arduplane" 2>/dev/null
pkill -9 -f run_in_terminal_window 2>/dev/null
echo "[xc] exit $RC"
exit $RC
