"""Plan a ground-side soaring route from a weather prior.

Greedy strategic chain: starting from home, repeatedly pick the best reachable
thermal candidate toward the goal (reusing `navigation.thermal_prior.BeliefMap`,
the same scoring the dashboard and companion use), step to it, and continue until
the goal vicinity is reached or candidates run out. The result is an ordered list
of waypoints we hand off as an uploadable path; the Pi 5 + flight controller fly
it.

Output formats:
  * route JSON   — our rich format (lat/lon + ENU + forecast W* + probability)
  * QGC .waypoints — standard Mission Planner / QGC route the Pi 5 can upload

CLI:
    python -m planner.route_planner --source soaringmeteo --lat 43.47 --lon -80.54
    python -m planner.route_planner --prior weather/data/soaringmeteo_prior_...json
"""
from __future__ import annotations

import argparse
import json
import math
import os

from navigation.thermal_prior import BeliefMap, CandidatePoint

_R_EARTH = 6378137.0  # WGS84, m


def enu_to_latlon(origin_lat, origin_lon, east_m, north_m):
    dlat = math.degrees(north_m / _R_EARTH)
    dlon = math.degrees(east_m / (_R_EARTH * math.cos(math.radians(origin_lat))))
    return origin_lat + dlat, origin_lon + dlon


def latlon_to_enu(origin_lat, origin_lon, lat, lon):
    north = math.radians(lat - origin_lat) * _R_EARTH
    east = math.radians(lon - origin_lon) * _R_EARTH * math.cos(math.radians(origin_lat))
    return east, north


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def build_region_prior(source, lat, lon, size_km, at_time=None, w_min=0.8):
    """Build a cross-country prior whose candidates are REAL W* grid cells over a
    size_km box (not a small-box sampled field). Each reachable cell with
    W* >= w_min becomes a candidate at its true ENU position relative to (lat,lon).
    """
    half_lat = (size_km / 2.0) / 111.0
    half_lon = (size_km / 2.0) / (111.0 * math.cos(math.radians(lat)))
    if source == "soaringmeteo":
        from weather.soaringmeteo import fetch_region
        meta, recs = fetch_region(lat - half_lat, lat + half_lat,
                                  lon - half_lon, lon + half_lon)
    else:
        import datetime as _dt
        from weather.openmeteo_thermal import fetch_region
        at_time = at_time or _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT18:00:00Z")
        meta, recs = fetch_region(lat - half_lat, lat + half_lat,
                                  lon - half_lon, lon + half_lon, at_time)
    cands, winds, base = [], [], []
    for r in recs:
        w = r["thermal_velocity_ms"]
        if w < w_min:
            continue
        e, n = latlon_to_enu(lat, lon, r["lat"], r["lon"])
        p = _clamp(0.4 + 0.1 * w, 0.2, 0.9)
        cands.append([round(e, 1), round(n, 1), round(w, 2), round(p, 2)])
        winds.append((r.get("wind_u_kmh", 0.0), r.get("wind_v_kmh", 0.0)))
        if r.get("soaring_layer_top_m"):
            base.append(r["soaring_layer_top_m"])
    wx = round(sum(w[0] for w in winds) / max(len(winds), 1) / 3.6, 2) if winds else 0.0
    wy = round(sum(w[1] for w in winds) / max(len(winds), 1) / 3.6, 2) if winds else 0.0
    extent = size_km * 1000.0 / 2.0
    return {
        "generated_at": meta.get("run", "latest"),
        "location": {"lat": lat, "lon": lon, "time": meta.get("time")},
        "bounds": [-extent, extent, -extent, extent],
        "wind": [wx, wy],
        "cloud_base_m": round(sum(base) / len(base)) if base else None,
        "thermal_strength_ms": round(max((c[2] for c in cands), default=0.0), 2),
        "thermal_count": len(cands),
        "candidates": cands,
        "source": f"{source}-region",
    }


def plan_route(prior, goal_enu=None, start_enu=(0.0, 0.0), plan_alt=1500.0,
               max_waypoints=8, arrive_m=120.0):
    """Return an ordered list of waypoints (dicts) from a weather prior.

    Each waypoint: {seq, enu_x, enu_y, w_star, prob}. Greedy: best reachable
    candidate toward the goal, mark it used, step there, repeat.
    """
    cands = [CandidatePoint(x=c[0], y=c[1], prob=c[3], strength_guess=c[2])
             for c in prior["candidates"]]
    belief = BeliefMap(cands)
    if goal_enu is None:
        # default goal = the strongest candidate (prob x strength). For a long
        # cross-country route, pass an explicit far --goal-lat/--goal-lon and the
        # chain bridges to it through intermediate thermals.
        g = max(cands, key=lambda c: c.prob * c.strength_guess)
        goal_enu = (g.x, g.y)

    route = []
    cur = start_enu
    for seq in range(max_waypoints):
        target = belief.best_target(cur[0], cur[1], plan_alt, goal_enu)
        if target is None:
            break
        route.append({"seq": seq + 1, "enu_x": round(target.x, 1),
                      "enu_y": round(target.y, 1),
                      "w_star": round(target.strength_guess, 2),
                      "prob": round(target.prob, 2)})
        target.confirmed = True            # mark used so we don't re-pick it
        cur = (target.x, target.y)
        if math.hypot(goal_enu[0] - cur[0], goal_enu[1] - cur[1]) <= arrive_m:
            break
    return route, goal_enu


def to_latlon_route(route, origin):
    """Attach lat/lon to each ENU waypoint."""
    out = []
    for wp in route:
        lat, lon = enu_to_latlon(origin[0], origin[1], wp["enu_x"], wp["enu_y"])
        out.append({**wp, "lat": round(lat, 7), "lon": round(lon, 7)})
    return out


def write_json(route_ll, prior, origin, goal_ll, path, takeoff_alt, ceiling_alt):
    doc = {
        "source": prior.get("source"),
        "generated_for_time": prior.get("location", {}).get("time"),
        "origin": {"lat": origin[0], "lon": origin[1]},
        "goal": {"lat": goal_ll[0], "lon": goal_ll[1]},
        "takeoff_alt_m": takeoff_alt,
        "ceiling_alt_m": ceiling_alt,
        "thermal_strength_ms": prior.get("thermal_strength_ms"),
        "cloud_base_m": prior.get("cloud_base_m"),
        "wind": prior.get("wind"),
        # each waypoint is a thermal hotspot to soar at (climb to ceiling, then go)
        "waypoints": [{**wp, "role": "thermal_loiter_to_alt"} for wp in route_ll],
        # what the Pi 5 should do with this path (FC is ArduPilot/ArduSoar)
        "handoff": {
            "autopilot": "ArduPilot (ArduSoar)",
            "upload": "the sibling .waypoints file is a native ArduPilot mission; upload as-is",
            "before_auto": ["set SOAR_ENABLE=1",
                            "enable soaring: MAV_CMD_DO_AUX_FUNCTION(88, HIGH)",
                            "set mode AUTO and arm"],
            "per_hotspot": "NAV_LOITER_TO_ALT lets ArduSoar circle and climb to ceiling_alt_m, then it glides to the next",
            "vision_return": "if the Pi reports vision-confirmed thermal positions, feed them back to re-plan (BeliefMap.confirm/disconfirm) and re-emit a route",
        },
    }
    with open(path, "w") as f:
        json.dump(doc, f, indent=2)
    return path


def write_qgc(route_ll, origin, path, takeoff_alt, ceiling_alt, home_alt=0.0,
              plain=False):
    """Write a native ArduPilot mission (QGC WPL 110).

    Soaring-aware (default): home, NAV_TAKEOFF, then each hotspot is a
    NAV_LOITER_TO_ALT that circles/soars up to ``ceiling_alt`` before gliding to
    the next, and a final NAV_RETURN_TO_LAUNCH. With SOAR_ENABLE=1 the loiters are
    where ArduSoar works the thermal; if there's no lift it climbs on the motor, so
    it never hangs. ``plain=True`` emits simple NAV_WAYPOINTs instead.

    Frames: 0=global (home), 3=global-relative-alt. Commands: 16 WAYPOINT,
    22 TAKEOFF, 31 LOITER_TO_ALT, 20 RETURN_TO_LAUNCH.
    """
    rows = ["QGC WPL 110"]

    def row(seq, cur, frame, cmd, p1, p2, lat, lon, alt):
        return (f"{seq}\t{cur}\t{frame}\t{cmd}\t{p1:.6f}\t{p2:.6f}\t0.000000\t"
                f"0.000000\t{lat:.8f}\t{lon:.8f}\t{alt:.6f}\t1")

    rows.append(row(0, 1, 0, 16, 0, 0, origin[0], origin[1], home_alt))
    rows.append(row(1, 0, 3, 22, 15.0, 0, origin[0], origin[1], takeoff_alt))
    seq = 2
    for wp in route_ll:
        if plain:
            rows.append(row(seq, 0, 3, 16, 0, 0, wp["lat"], wp["lon"], ceiling_alt))
        else:
            # NAV_LOITER_TO_ALT: p1=heading-required, p2=loiter radius (0=default)
            rows.append(row(seq, 0, 3, 31, 1.0, 0, wp["lat"], wp["lon"], ceiling_alt))
        seq += 1
    rows.append(row(seq, 0, 2, 20, 0, 0, 0.0, 0.0, 0.0))     # RTL
    with open(path, "w") as f:
        f.write("\n".join(rows) + "\n")
    return path


def _load_prior(args):
    if args.prior:
        with open(args.prior) as f:
            return json.load(f)
    if args.source == "soaringmeteo":
        from weather.soaringmeteo_prior import build_prior
        return build_prior(args.lat, args.lon)
    if args.source == "openmeteo":
        from weather.openmeteo_prior import build_prior
        return build_prior(args.lat, args.lon)
    raise SystemExit("provide --prior FILE or --source {soaringmeteo,openmeteo} --lat --lon")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--prior", help="a prior JSON file (overrides --source)")
    ap.add_argument("--source", choices=["soaringmeteo", "openmeteo"], default="soaringmeteo")
    ap.add_argument("--lat", type=float, default=43.47)
    ap.add_argument("--lon", type=float, default=-80.54)
    ap.add_argument("--region-km", type=float, default=None,
                    help="plan cross-country over a real W* grid of this size (km) instead of a local box")
    ap.add_argument("--w-min", type=float, default=0.8, help="min W* (m/s) for a region cell to be a candidate")
    ap.add_argument("--goal-lat", type=float, default=None)
    ap.add_argument("--goal-lon", type=float, default=None)
    ap.add_argument("--takeoff-alt", type=float, default=120.0)
    ap.add_argument("--ceiling-alt", type=float, default=None,
                    help="climb-to altitude at each thermal (default: cloud_base - 200 m)")
    ap.add_argument("--plain", action="store_true",
                    help="emit plain NAV_WAYPOINTs instead of soaring NAV_LOITER_TO_ALT")
    ap.add_argument("--max-waypoints", type=int, default=8)
    ap.add_argument("--out-dir", default=os.path.join(os.path.dirname(__file__), "routes"))
    args = ap.parse_args()

    if args.region_km and not args.prior:
        prior = build_region_prior(args.source, args.lat, args.lon, args.region_km, w_min=args.w_min)
    else:
        prior = _load_prior(args)
    loc = prior["location"]
    origin = (loc["lat"], loc["lon"])

    goal_enu = None
    if args.goal_lat is not None and args.goal_lon is not None:
        goal_enu = latlon_to_enu(origin[0], origin[1], args.goal_lat, args.goal_lon)

    route, goal_enu = plan_route(prior, goal_enu=goal_enu, plan_alt=1500.0,
                                 max_waypoints=args.max_waypoints)
    route_ll = to_latlon_route(route, origin)
    goal_ll = enu_to_latlon(origin[0], origin[1], goal_enu[0], goal_enu[1])

    # soaring ceiling: climb to just under cloud base at each thermal
    if args.ceiling_alt is not None:
        ceiling = args.ceiling_alt
    else:
        base = prior.get("cloud_base_m") or 600.0
        ceiling = _clamp(base - 200.0, args.takeoff_alt + 50.0, 3000.0)

    os.makedirs(args.out_dir, exist_ok=True)
    tag = f"{prior.get('source','route')}_{args.lat}_{args.lon}"
    jpath = write_json(route_ll, prior, origin, goal_ll,
                       os.path.join(args.out_dir, f"route_{tag}.json"),
                       args.takeoff_alt, round(ceiling))
    wpath = write_qgc(route_ll, origin,
                      os.path.join(args.out_dir, f"route_{tag}.waypoints"),
                      args.takeoff_alt, round(ceiling), plain=args.plain)

    print(f"Ground route from {prior.get('source')}  origin {origin[0]:.5f},{origin[1]:.5f}  "
          f"W*~{prior.get('thermal_strength_ms')} m/s  ceiling {round(ceiling)} m")
    print(f"  goal: {goal_ll[0]:.5f},{goal_ll[1]:.5f}   {len(route_ll)} waypoints:")
    for wp in route_ll:
        print(f"   {wp['seq']:>2}. {wp['lat']:.5f},{wp['lon']:.5f}  "
              f"W*={wp['w_star']:.1f} p={wp['prob']:.2f}  (ENU {wp['enu_x']:.0f},{wp['enu_y']:.0f})")
    print(f"  -> {jpath}")
    print(f"  -> {wpath}  (QGC .waypoints for the Pi 5 to upload)")
    return route_ll


if __name__ == "__main__":
    main()
