#!/usr/bin/env bash
# One-shot: launch a fresh plane-soaring SITL, run the weather-guided companion
# (fly GUIDED to the forecast hotspot, hand off to ArduSoar), then tear down.
#
# Usage: companion/run_companion_demo.sh
set -uo pipefail

VENV=/Users/momo/Downloads/dd/soar-venv/bin/python
ARDUPILOT=/Users/momo/Downloads/dd/ardupilot
REPO="$(cd "$(dirname "$0")/.." && pwd)"
HERE="$REPO/companion"
SITL_LOG=/tmp/sitl_companion.log

echo "[companion] killing any stale SITL"
pkill -9 -f "sim_vehicle.py" 2>/dev/null
pkill -9 -f "build/sitl/bin/arduplane" 2>/dev/null
pkill -9 -f run_in_terminal_window 2>/dev/null
sleep 2

echo "[companion] generating SITL-aligned weather prior"
$VENV "$HERE/make_sitl_prior.py"

echo "[companion] launching fresh plane-soaring SITL"
cd "$ARDUPILOT"
$VENV Tools/autotest/sim_vehicle.py -v ArduPlane -f plane-soaring \
    --no-mavproxy --no-rebuild -w --speedup 20 > "$SITL_LOG" 2>&1 &

echo "[companion] waiting for TCP 5760"
for i in $(seq 1 60); do
    if lsof -nP -iTCP:5760 -sTCP:LISTEN >/dev/null 2>&1; then
        echo "[companion] SITL listening (after ${i}s)"; break
    fi
    sleep 1
done
sleep 5

echo "[companion] running weather-guided companion"
cd "$REPO"
$VENV "$HERE/weather_guided_companion.py" --conn tcp:127.0.0.1:5760 \
    --prior "$HERE/sitl_prior.json" --origin home --timeout 900
RC=$?

echo "[companion] tearing down SITL"
pkill -9 -f "sim_vehicle.py" 2>/dev/null
pkill -9 -f "build/sitl/bin/arduplane" 2>/dev/null
pkill -9 -f run_in_terminal_window 2>/dev/null

echo "[companion] companion exit code: $RC"
exit $RC
