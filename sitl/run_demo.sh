#!/usr/bin/env bash
# One-shot: launch a fresh plane-soaring SITL, run the ArduSoar thermalling demo
# against it over MAVLink, then tear the SITL down.
#
# Usage: sitl/run_demo.sh
set -uo pipefail

VENV=/Users/momo/Downloads/dd/soar-venv/bin/python
ARDUPILOT=/Users/momo/Downloads/dd/ardupilot
MISSION="$ARDUPILOT/Tools/autotest/ArduPlane_Tests/Soaring/CMAC-soar.txt"
HERE="$(cd "$(dirname "$0")" && pwd)"
SITL_LOG=/tmp/sitl_soaring.log

echo "[run_demo] killing any stale SITL"
pkill -f "sim_vehicle.py" 2>/dev/null
pkill -f "build/sitl/bin/arduplane" 2>/dev/null
sleep 2

echo "[run_demo] launching fresh plane-soaring SITL (wiped, on ground)"
cd "$ARDUPILOT"
$VENV Tools/autotest/sim_vehicle.py -v ArduPlane -f plane-soaring \
    --no-mavproxy --no-rebuild -w --speedup 20 > "$SITL_LOG" 2>&1 &
SITL_PID=$!

echo "[run_demo] waiting for TCP 5760 to listen"
for i in $(seq 1 60); do
    if lsof -nP -iTCP:5760 -sTCP:LISTEN >/dev/null 2>&1; then
        echo "[run_demo] SITL listening (after ${i}s)"
        break
    fi
    sleep 1
done

# Give the EKF a moment to settle before the demo connects.
sleep 5

echo "[run_demo] running soaring demo"
$VENV "$HERE/run_soaring_demo.py" --conn tcp:127.0.0.1:5760 --mission "$MISSION" --timeout 900
RC=$?

echo "[run_demo] tearing down SITL"
pkill -f "sim_vehicle.py" 2>/dev/null
pkill -f "build/sitl/bin/arduplane" 2>/dev/null

echo "[run_demo] demo exit code: $RC"
exit $RC
