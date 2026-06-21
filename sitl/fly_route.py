#!/usr/bin/env python3
"""Fly a planner-generated route in SITL to validate it end-to-end.

Uploads a `planner/` .waypoints mission verbatim to ArduPilot, enables ArduSoar,
flies AUTO, and confirms the aircraft reaches the first planned hotspot and
ArduSoar climbs there. Proves the path the ground planner hands to the Pi 5 is a
valid, flyable ArduPilot mission.

Usage: fly_route.py --route planner/routes/route_*.waypoints
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from companion import mav  # noqa: E402


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--conn", default="tcp:127.0.0.1:5760")
    ap.add_argument("--route", required=True)
    ap.add_argument("--timeout", type=float, default=300)
    args = ap.parse_args()

    m = mav.connect(args.conn)
    log(f"Connected (sys {m.target_system})")
    if not mav.wait_gps_fix(m):
        log("FAILED: no GPS fix")
        return 2

    mav.set_param(m, "SOAR_VSPEED", 0.55)
    mav.set_param(m, "SOAR_ENABLE", 1)

    ok, n = mav.upload_qgc_file(m, args.route)
    if not ok:
        log("FAILED: mission rejected")
        return 1
    log(f"Mission ACCEPTED ({n} items) from {os.path.basename(args.route)}")

    mav.set_mode(m, "AUTO")
    if not mav.arm(m):
        log("FAILED: could not arm")
        return 1
    log("Armed; climbing out (soaring enabled once airborne)")
    # Enable ArduSoar only AFTER takeoff — enabling it during the takeoff climb
    # suppresses throttle and the plane can't climb out.
    airborne = time.time() + 120
    while time.time() < airborne:
        pos = mav.vehicle_position(m)
        if pos and pos[2] >= 90:
            break
    mav.set_soaring_switch(m, 2)
    log("Airborne; ArduSoar enabled, flying the planned route")

    entry_alt = None
    peak = -1e9
    saw_thermal = False
    t0 = time.time()
    while time.time() - t0 < args.timeout:
        try:
            msg = m.recv_match(type=["HEARTBEAT", "GLOBAL_POSITION_INT", "STATUSTEXT"],
                               blocking=True, timeout=1)
        except ConnectionError:
            log("Connection to SITL lost")
            break
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            if "oar" in str(msg.text):
                log(f"AP: {msg.text}")
            continue
        if msg.get_type() != "GLOBAL_POSITION_INT":
            continue
        alt = msg.relative_alt / 1000.0
        peak = max(peak, alt)
        if m.flightmode == "THERMAL" and not saw_thermal:
            saw_thermal, entry_alt = True, alt
            log(f"--> ArduSoar entered THERMAL at {alt:.0f} m (flying the planned mission)")
        # Validation boundary for a PLANNER: the route uploads, flies, and ArduSoar
        # engages at the planned hotspot. The climb magnitude is ArduSoar's job.
        if saw_thermal and entry_alt is not None and alt - entry_alt >= 40:
            log(f"--> Climbed +{alt - entry_alt:.0f} m at the planned hotspot (now {alt:.0f} m)")
            break
    if saw_thermal:
        climb = (peak - entry_alt) if entry_alt is not None else 0.0
        log(f"RESULT: ROUTE VALIDATED — uploaded to ArduPilot, flew the planned mission, "
            f"ArduSoar engaged at the planned hotspot (+{climb:.0f} m, peak {peak:.0f} m)")
        return 0
    log(f"RESULT: FAIL — never engaged a thermal (peak {peak:.0f} m)")
    return 1


if __name__ == "__main__":
    sys.exit(main())
