#!/usr/bin/env python3
"""Weather-guided cross-country: hop between forecast thermals toward a goal.

Multi-hotspot version of `weather_guided_companion.py`. The companion repeatedly
picks the best reachable candidate toward a goal (reusing
`navigation.thermal_prior.BeliefMap`), flies the aircraft there over MAVLink, and
hands off to ArduSoar. On each arrival it watches for lift and **confirms**
(climbed) or **disconfirms** (searched, nothing) the candidate in the belief map,
then hops to the next — exactly the strategic relay the dashboard shows in sim,
now driving a real autopilot.

SITL note: ArduPilot SITL has a single synthetic thermal, so only the aligned
candidate actually climbs; the other hops demonstrate the fly -> search ->
disconfirm -> re-target machinery. A true multi-climb cross-country needs custom
SITL thermals or hardware (or watch it in the dashboard on real weather).
"""
import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from navigation.thermal_prior import BeliefMap, CandidatePoint  # noqa: E402
from companion import geo, mav  # noqa: E402


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--conn", default="tcp:127.0.0.1:5760")
    ap.add_argument("--prior", default=os.path.join(os.path.dirname(__file__), "sitl_prior.json"))
    ap.add_argument("--origin", choices=["home", "prior"], default="home")
    ap.add_argument("--takeoff-alt", type=float, default=120.0)
    ap.add_argument("--cruise-alt", type=float, default=120.0)
    ap.add_argument("--capture-radius", type=float, default=90.0)
    ap.add_argument("--max-hops", type=int, default=3)
    ap.add_argument("--confirm-climb", type=float, default=40.0, help="m of climb to confirm a thermal")
    ap.add_argument("--nav-timeout", type=float, default=240)
    ap.add_argument("--search-window", type=float, default=60, help="s to watch for lift after handoff")
    args = ap.parse_args()

    log(f"Connecting to {args.conn}")
    m = mav.connect(args.conn)
    if not mav.wait_gps_fix(m):
        log("FAILED: no GPS fix")
        return 2
    home = mav.get_home(m)
    log(f"Home: {home[0]:.6f}, {home[1]:.6f}")

    with open(args.prior) as f:
        prior = json.load(f)
    cands = [CandidatePoint(x=c[0], y=c[1], prob=c[3], strength_guess=c[2])
             for c in prior["candidates"]]
    loc = prior.get("location", {})
    origin = home if args.origin == "home" else (loc["lat"], loc["lon"])
    belief = BeliefMap(cands)
    goal = max(cands, key=lambda c: c.prob * c.strength_guess)
    goal_xy = (goal.x, goal.y)
    log(f"Origin ({args.origin}): {origin[0]:.6f},{origin[1]:.6f} | {len(cands)} candidates | "
        f"goal ENU ({goal.x:.0f},{goal.y:.0f})")

    mav.set_param(m, "SOAR_VSPEED", 0.55)
    mav.set_param(m, "SOAR_ENABLE", 1)

    cur = (0.0, 0.0)          # companion's ENU position (starts at home)
    plan_alt = args.cruise_alt
    armed = False
    hops = []

    for hop in range(args.max_hops):
        target = belief.best_target(cur[0], cur[1], 1500.0, goal_xy)  # box fits one glide
        if target is None:
            log("No more reachable candidates")
            break
        tlat, tlon = geo.enu_to_latlon(origin[0], origin[1], target.x, target.y)
        log(f"--- Hop {hop+1}: -> ENU ({target.x:.0f},{target.y:.0f}) "
            f"W*={target.strength_guess:.1f} p={target.prob:.2f}  {tlat:.5f},{tlon:.5f}")

        # navigate there
        if not armed:
            mav.upload_hotspot_mission(m, args.takeoff_alt, (tlat, tlon), args.cruise_alt)
            mav.set_mode(m, "AUTO")
            if not mav.arm(m):
                log("FAILED: could not arm")
                return 1
            armed = True
            log("Armed; flying first hop")
        else:
            mav.upload_goto_mission(m, (tlat, tlon), args.cruise_alt)
            mav.set_mode(m, "AUTO")
            mav.set_current_wp(m, 1)
            log("Re-targeted to next hop")

        # wait until we reach the hotspot
        t0 = time.time()
        arrived = False
        while time.time() - t0 < args.nav_timeout:
            pos = mav.vehicle_position(m, timeout=1)
            if pos is None:
                continue
            d = geo.haversine_m(pos[0], pos[1], tlat, tlon)
            if pos[2] >= args.takeoff_alt * 0.7 and d <= args.capture_radius:
                arrived = True
                break
        if not arrived:
            log(f"  did not reach hop {hop+1}; skipping")
            belief.disconfirm(target)
            cur = (target.x, target.y)
            continue

        # hand off and watch for lift
        mav.set_soaring_switch(m, 2)
        log(f"  reached; handed off to ArduSoar, watching {args.search_window:.0f}s for lift")
        entry_alt = None
        peak = -1e9
        saw_thermal = False
        ts = time.time()
        while time.time() - ts < args.search_window:
            try:
                msg = m.recv_match(type=["HEARTBEAT", "GLOBAL_POSITION_INT", "STATUSTEXT"],
                                   blocking=True, timeout=1)
            except ConnectionError:
                break
            if msg is None:
                continue
            if msg.get_type() == "STATUSTEXT":
                if "oar" in str(msg.text):
                    log(f"  AP: {msg.text}")
                continue
            if msg.get_type() != "GLOBAL_POSITION_INT":
                continue
            alt = msg.relative_alt / 1000.0
            peak = max(peak, alt)
            if m.flightmode == "THERMAL" and not saw_thermal:
                saw_thermal, entry_alt = True, alt
            if saw_thermal and entry_alt is not None and alt - entry_alt >= args.confirm_climb:
                break

        climbed = (peak - entry_alt) if entry_alt is not None else 0.0
        if saw_thermal and climbed >= args.confirm_climb:
            belief.confirm(target, target.x, target.y, target.strength_guess)
            log(f"  CONFIRMED: climbed +{climbed:.0f} m (now {peak:.0f} m), prob->{target.prob:.2f}")
            hops.append((hop + 1, "CONFIRMED", climbed))
        else:
            belief.disconfirm(target)
            log(f"  disconfirmed (no usable lift), prob->{target.prob:.2f}")
            hops.append((hop + 1, "disconfirmed", climbed))
        cur = (target.x, target.y)

    log("=" * 60)
    confirmed = [h for h in hops if h[1] == "CONFIRMED"]
    log(f"Cross-country relay: {len(hops)} hops, {len(confirmed)} confirmed thermal(s)")
    for n, status, climbed in hops:
        log(f"  hop {n}: {status:13s} climb {climbed:.0f} m")
    log("=" * 60)
    return 0 if confirmed else 1


if __name__ == "__main__":
    sys.exit(main())
