#!/usr/bin/env python3
"""Weather-guided companion: fly the aircraft to the best forecast thermal hotspot
over MAVLink (GUIDED), then hand off to ArduPilot's ArduSoar to circle and climb.

Strategic layer (ours)            Tactical layer (ArduSoar, onboard)
---------------------------       ----------------------------------
weather prior -> best hotspot     detect lift, enter THERMAL, circle core
GUIDED waypoint there             climb toward SOAR_ALT_MAX
hand off (FBWB + soaring on)  --> ArduSoar takes it from here

Selection reuses the repo's existing strategic assets (navigation.thermal_prior
.BeliefMap), so the "where are today's thermals" brain is shared with the
simulator. Tested against ArduPilot SITL; see companion/README.md.
"""
import argparse
import json
import os
import sys
import time

# repo root on path so we can reuse the strategic layer
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from navigation.thermal_prior import BeliefMap, CandidatePoint  # noqa: E402
from companion import geo, mav  # noqa: E402


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def load_candidates(prior_path):
    with open(prior_path) as f:
        prior = json.load(f)
    cands = [CandidatePoint(x=c[0], y=c[1], prob=c[3], strength_guess=c[2])
             for c in prior["candidates"]]
    loc = prior.get("location", {})
    wind = prior.get("wind", [0.0, 0.0])
    return cands, (loc.get("lat"), loc.get("lon")), wind


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--conn", default="tcp:127.0.0.1:5760")
    ap.add_argument("--prior", default=os.path.join(os.path.dirname(__file__), "sitl_prior.json"))
    ap.add_argument("--origin", choices=["home", "prior"], default="home",
                    help="interpret candidate metres relative to live home (SITL) or prior.location (field)")
    ap.add_argument("--takeoff-alt", type=float, default=120.0)
    ap.add_argument("--cruise-alt", type=float, default=120.0)
    ap.add_argument("--capture-radius", type=float, default=90.0)
    ap.add_argument("--nav-timeout", type=float, default=240)
    ap.add_argument("--timeout", type=float, default=900)
    args = ap.parse_args()

    log(f"Connecting to {args.conn}")
    m = mav.connect(args.conn)
    log(f"Heartbeat from system {m.target_system}")

    log("Waiting for GPS 3D fix")
    if not mav.wait_gps_fix(m):
        log("FAILED: no GPS fix")
        return 2
    home = mav.get_home(m)
    if home is None:
        log("FAILED: no home position")
        return 2
    log(f"Home: {home[0]:.6f}, {home[1]:.6f}")

    cands, prior_loc, wind = load_candidates(args.prior)
    origin = home if args.origin == "home" else prior_loc
    log(f"Origin ({args.origin}): {origin[0]:.6f}, {origin[1]:.6f}  | {len(cands)} candidates")

    # Strategic selection: best reachable, strong, likely candidate.
    belief = BeliefMap(cands)
    goal = max(cands, key=lambda c: c.prob * c.strength_guess)
    chosen = belief.best_target(0.0, 0.0, args.cruise_alt, (goal.x, goal.y))
    if chosen is None:
        log("FAILED: no reachable candidate")
        return 1
    tlat, tlon = geo.enu_to_latlon(origin[0], origin[1], chosen.x, chosen.y)
    log(f"Chosen hotspot: ENU ({chosen.x:.0f},{chosen.y:.0f}) m  "
        f"W*={chosen.strength_guess:.1f} p={chosen.prob:.2f}  -> {tlat:.6f},{tlon:.6f}")

    # Soaring tuning (match the SITL milestone-1 demo).
    mav.set_param(m, "SOAR_VSPEED", 0.55)
    mav.set_param(m, "SOAR_ENABLE", 1)

    # --- Fly to the forecast hotspot under an AUTO mission (takeoff -> waypoint
    #     -> loiter at the hotspot). The companion picked WHERE; the autopilot
    #     flies the aircraft there. ---
    log("Uploading hotspot mission (takeoff -> waypoint -> loiter @ hotspot)")
    if not mav.upload_hotspot_mission(m, args.takeoff_alt, (tlat, tlon), args.cruise_alt):
        log("FAILED: mission upload")
        return 1
    mav.set_mode(m, "AUTO")
    if not mav.wait_gps_fix(m):
        log("FAILED: no GPS fix")
        return 1
    if not mav.arm(m):
        log("FAILED: could not arm")
        return 1
    log("Armed; flying mission to hotspot")
    t0 = time.time()
    arrived = False
    last_log = 0.0
    while time.time() - t0 < args.nav_timeout:
        pos = mav.vehicle_position(m, timeout=1)
        if pos is None:
            continue
        d = geo.haversine_m(pos[0], pos[1], tlat, tlon)
        if time.time() - last_log >= 3:
            log(f"  enroute[{m.flightmode}]: {d:6.0f} m to hotspot, alt {pos[2]:.0f} m")
            last_log = time.time()
        if pos[2] >= args.takeoff_alt * 0.7 and d <= args.capture_radius:
            log(f"--> Reached hotspot ({d:.0f} m), handing off to ArduSoar")
            arrived = True
            break
    if not arrived:
        log("FAILED: never reached hotspot")
        return 1

    # --- HANDOFF: enable soaring and let ArduSoar take over. We stay in AUTO:
    #     the mission's final item loiters at the hotspot, so the aircraft keeps
    #     circling inside the thermal while ArduSoar detects lift and switches to
    #     THERMAL to centre and climb. (FBWB would fly straight out of it.) ---
    mav.set_soaring_switch(m, 2)  # enable soaring (auto mode changes)
    log("Handed off: AUTO loiter @ hotspot + soaring enabled; watching for THERMAL + climb")

    entry_alt = None
    peak = -1e9
    saw_thermal = False
    result = "DISCONFIRMED"
    t1 = time.time()
    while time.time() - t1 < args.timeout:
        try:
            msg = m.recv_match(type=["HEARTBEAT", "GLOBAL_POSITION_INT", "STATUSTEXT"],
                               blocking=True, timeout=1)
        except ConnectionError:
            log("Connection to SITL lost during handoff monitor")
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
        mode = m.flightmode
        peak = max(peak, alt)
        if mode == "THERMAL" and not saw_thermal:
            saw_thermal, entry_alt = True, alt
            log(f"--> ArduSoar entered THERMAL at {alt:.0f} m")
        if saw_thermal and entry_alt is not None and alt >= entry_alt + 60:
            log(f"--> Climbed +{alt - entry_alt:.0f} m in the thermal (now {alt:.0f} m)")
            result = "CONFIRMED"
            break

    # Strategic-map feedback: confirm or disconfirm this candidate.
    if result == "CONFIRMED":
        belief.confirm(chosen, chosen.x, chosen.y, chosen.strength_guess)
    else:
        belief.disconfirm(chosen)

    log("=" * 60)
    log(f"RESULT: {result}")
    log(f"  reached forecast hotspot:  True")
    log(f"  ArduSoar entered THERMAL:  {saw_thermal}")
    if entry_alt is not None:
        log(f"  climb in thermal:          {peak - entry_alt:.0f} m (entry {entry_alt:.0f} -> peak {peak:.0f})")
    log(f"  candidate prob after flight: {chosen.prob:.2f}")
    log("=" * 60)
    return 0 if result == "CONFIRMED" else 1


if __name__ == "__main__":
    sys.exit(main())
