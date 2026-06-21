#!/usr/bin/env python3
"""
Reproduce ArduPilot's ArduSoar thermalling in pure software (SITL), driven over
MAVLink with pymavlink.

This is Milestone 1 of the ArduSoar pivot and the seed of the step-3 weather
companion: the same connection/upload/monitor pattern the companion will use to
push GUIDED waypoints lives here.

It mirrors the official `Tools/autotest/arduplane.py::Soaring` logic but does a
single clean mission upload (the autotest's overlapping fence+mission upload
races on macOS), then watches the vehicle:

    AUTO cruise  ->  SOAR triggers on rising air  ->  THERMAL (LOITER) circling
                 ->  climbs toward SOAR_ALT_MAX    ->  back to AUTO

Prereq: a SITL instance already running and listening, e.g.
    sim_vehicle.py -v ArduPlane -f plane-soaring --no-mavproxy --no-rebuild -w --speedup 20

Usage:
    run_soaring_demo.py --conn tcp:127.0.0.1:5760 --mission <CMAC-soar.txt>
"""
import argparse
import csv
import os
import sys
import time

from pymavlink import mavutil


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def load_qgc_mission(path):
    """Parse a QGC WPL 110 .txt waypoint file into a list of dicts."""
    items = []
    with open(path) as f:
        header = f.readline().strip()
        if not header.startswith("QGC WPL"):
            raise ValueError(f"not a QGC WPL file: {header!r}")
        for line in f:
            line = line.strip()
            if not line:
                continue
            c = line.split("\t")
            items.append(dict(
                seq=int(c[0]), current=int(c[1]), frame=int(c[2]), command=int(c[3]),
                p1=float(c[4]), p2=float(c[5]), p3=float(c[6]), p4=float(c[7]),
                x=float(c[8]), y=float(c[9]), z=float(c[10]), autocont=int(c[11]),
            ))
    return items


def upload_mission(m, items):
    """Upload a mission with a single MISSION_COUNT/REQUEST/ITEM_INT exchange."""
    log(f"Uploading {len(items)} mission items")
    m.mav.mission_count_send(m.target_system, m.target_component,
                             len(items), mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    sent = set()
    deadline = time.time() + 30
    while len(sent) < len(items) and time.time() < deadline:
        msg = m.recv_match(type=["MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"],
                           blocking=True, timeout=5)
        if msg is None:
            continue
        t = msg.get_type()
        if t == "MISSION_ACK":
            if msg.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                raise RuntimeError(f"mission upload rejected: ack type {msg.type}")
            break
        seq = msg.seq
        it = items[seq]
        m.mav.mission_item_int_send(
            m.target_system, m.target_component, seq, it["frame"], it["command"],
            0, it["autocont"], it["p1"], it["p2"], it["p3"], it["p4"],
            int(it["x"] * 1e7), int(it["y"] * 1e7), it["z"],
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        sent.add(seq)
    # final ack
    ack = m.recv_match(type="MISSION_ACK", blocking=True, timeout=10)
    if ack is None or ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
        raise RuntimeError(f"no clean mission ACK (got {ack})")
    log("Mission accepted")


def set_param(m, name, value):
    m.mav.param_set_send(m.target_system, m.target_component,
                         name.encode(), float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    # confirm
    deadline = time.time() + 5
    while time.time() < deadline:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
        if msg and msg.param_id.strip("\x00") == name:
            return msg.param_value
    return None


def get_param(m, name):
    m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
    deadline = time.time() + 5
    while time.time() < deadline:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
        if msg and msg.param_id.strip("\x00") == name:
            return msg.param_value
    return None


SOARING_AUX_FUNC = 88  # RCx_OPTION value for "Soaring Enable"


def set_soaring_switch(m, level):
    """Invoke the SOARING aux function directly (0=LOW/disabled,
    1=MIDDLE/manual, 2=HIGH/auto) via MAV_CMD_DO_AUX_FUNCTION.

    This bypasses RC-channel values: in headless SITL the soaring-enable RC
    switch boots LOW, latching _pilot_desired_state=SOARING_DISABLED, and a
    plain RC override never reaches the aux-switch logic. The command does."""
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_DO_AUX_FUNCTION, 0,
                            SOARING_AUX_FUNC, level, 0, 0, 0, 0, 0)


def set_mode(m, mode_name):
    mapping = m.mode_mapping()
    if mode_name not in mapping:
        raise RuntimeError(f"unknown mode {mode_name}; have {list(mapping)}")
    m.set_mode(mapping[mode_name])


def arm(m):
    for attempt in range(60):
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                0, 1, 0, 0, 0, 0, 0, 0)
        ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=2)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                log("Armed")
                return True
        time.sleep(1)
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--conn", default="tcp:127.0.0.1:5760")
    ap.add_argument("--mission", required=True)
    ap.add_argument("--rc-chan", type=int, default=7, help="RC channel with SOAR enable (option 88)")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "soaring_log.csv"))
    ap.add_argument("--timeout", type=float, default=900, help="wall-clock seconds")
    args = ap.parse_args()

    log(f"Connecting to {args.conn}")
    m = mavutil.mavlink_connection(args.conn)
    m.wait_heartbeat()
    log(f"Heartbeat from system {m.target_system} component {m.target_component}")
    m.mav.request_data_stream_send(m.target_system, m.target_component,
                                   mavutil.mavlink.MAV_DATA_STREAM_ALL, 5, 1)

    # Match the official Soaring test tuning.
    set_param(m, "SOAR_VSPEED", 0.55)
    set_param(m, "SOAR_MIN_THML_S", 25)
    set_param(m, "SOAR_ENABLE", 1)
    alt_max = get_param(m, "SOAR_ALT_MAX") or 350.0
    log(f"SOAR_ALT_MAX = {alt_max:.0f} m (relative), SOAR_ENABLE = {get_param(m, 'SOAR_ENABLE')}")
    # Enable soaring with automatic mode changes (HIGH switch position).
    set_soaring_switch(m, 2)

    upload_mission(m, load_qgc_mission(args.mission))

    log("Waiting for GPS 3D fix / ready to arm")
    deadline = time.time() + 120
    while time.time() < deadline:
        g = m.recv_match(type="GPS_RAW_INT", blocking=True, timeout=2)
        if g and g.fix_type >= 3:
            break
    set_mode(m, "AUTO")
    set_soaring_switch(m, 2)  # re-assert after mode change
    if not arm(m):
        log("FAILED: could not arm")
        return 2
    set_soaring_switch(m, 2)

    # Monitor loop -------------------------------------------------------
    log("Monitoring: waiting for THERMAL mode and climb")
    samples = []          # (t, mode, rel_alt_m)
    t0 = time.time()
    saw_thermal = False
    thermal_entry_alt = None
    peak_alt = -1e9
    result = "TIMEOUT"
    while time.time() - t0 < args.timeout:
        msg = m.recv_match(type=["HEARTBEAT", "GLOBAL_POSITION_INT", "STATUSTEXT"],
                           blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            if "oar" in str(msg.text):  # "Soaring: ..." messages
                log(f"AP: {msg.text}")
            continue
        mode = m.flightmode
        if msg.get_type() == "GLOBAL_POSITION_INT":
            rel_alt = msg.relative_alt / 1000.0
            t = time.time() - t0
            samples.append((t, mode, rel_alt))
            peak_alt = max(peak_alt, rel_alt)
            if mode == "THERMAL" and not saw_thermal:
                saw_thermal = True
                thermal_entry_alt = rel_alt
                log(f"--> Entered THERMAL at {rel_alt:.0f} m, t={t:.0f}s")
            if saw_thermal and rel_alt >= alt_max - 15:
                log(f"--> Climbed to {rel_alt:.0f} m (>= SOAR_ALT_MAX-15), t={t:.0f}s")
                result = "PASS"
                break
            if len(samples) % 40 == 0:
                log(f"t={t:5.0f}s  mode={mode:8s}  alt={rel_alt:6.1f} m")

    # Results ------------------------------------------------------------
    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "mode", "rel_alt_m"])
        w.writerows(samples)
    log(f"Wrote {len(samples)} samples to {args.out}")

    log("=" * 60)
    log(f"RESULT: {result}")
    log(f"  entered THERMAL:    {saw_thermal}")
    if thermal_entry_alt is not None:
        log(f"  thermal entry alt:  {thermal_entry_alt:.0f} m")
    log(f"  peak relative alt:  {peak_alt:.0f} m  (SOAR_ALT_MAX={alt_max:.0f})")
    log("=" * 60)
    return 0 if result == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
