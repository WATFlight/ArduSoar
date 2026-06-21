"""Thin pymavlink helpers shared by the weather companion.

Same connection/command vocabulary proven in `sitl/run_soaring_demo.py`, packaged
for reuse: connect, params, modes, arm, a programmatic takeoff mission, GUIDED
goto, and the soaring-enable aux command.
"""
import time

from pymavlink import mavutil

SOARING_AUX_FUNC = 88  # RCx_OPTION value for "Soaring Enable"


def connect(conn_str):
    m = mavutil.mavlink_connection(conn_str)
    m.wait_heartbeat()
    m.mav.request_data_stream_send(m.target_system, m.target_component,
                                   mavutil.mavlink.MAV_DATA_STREAM_ALL, 5, 1)
    return m


def set_param(m, name, value):
    m.mav.param_set_send(m.target_system, m.target_component,
                         name.encode(), float(value),
                         mavutil.mavlink.MAV_PARAM_TYPE_REAL32)


def set_mode(m, mode_name):
    mapping = m.mode_mapping()
    if mode_name not in mapping:
        raise RuntimeError(f"unknown mode {mode_name}; have {list(mapping)}")
    m.set_mode(mapping[mode_name])


def set_soaring_switch(m, level):
    """0=LOW/disabled, 1=MIDDLE/manual, 2=HIGH/auto-mode-changes.

    Headless SITL boots the soaring RC switch LOW (=disabled) and a plain RC
    override doesn't reach the aux logic; this command does. See sitl/README.md.
    """
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_DO_AUX_FUNCTION, 0,
                            SOARING_AUX_FUNC, level, 0, 0, 0, 0, 0)


def arm(m, timeout=60):
    deadline = time.time() + timeout
    while time.time() < deadline:
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                0, 1, 0, 0, 0, 0, 0, 0)
        ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=2)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM \
                and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            return True
        time.sleep(1)
    return False


def wait_gps_fix(m, timeout=120):
    deadline = time.time() + timeout
    while time.time() < deadline:
        g = m.recv_match(type="GPS_RAW_INT", blocking=True, timeout=2)
        if g and g.fix_type >= 3:
            return True
    return False


def get_home(m, timeout=30):
    """Return (lat, lon) of home in degrees, from HOME_POSITION or first fix."""
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
                            0, 0, 0, 0, 0, 0, 0, 0)
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = m.recv_match(type=["HOME_POSITION", "GLOBAL_POSITION_INT"],
                           blocking=True, timeout=2)
        if msg is None:
            continue
        if msg.get_type() == "HOME_POSITION" and abs(msg.latitude) > 1_000_000:
            return msg.latitude / 1e7, msg.longitude / 1e7
        # Reject the pre-EKF "null island" (~0,0) value; require > ~0.1 deg.
        if msg.get_type() == "GLOBAL_POSITION_INT" and abs(msg.lat) > 1_000_000:
            return msg.lat / 1e7, msg.lon / 1e7
    return None


def upload_hotspot_mission(m, takeoff_alt, hotspot, cruise_alt):
    """AUTO mission that flies to a forecast hotspot and loiters there:
    home, NAV_TAKEOFF, NAV_WAYPOINT@hotspot, NAV_LOITER_UNLIM@hotspot.

    hotspot: (lat, lon). This is the companion's strategic command — the
    autopilot flies the aircraft to today's best thermal; ArduSoar takes the
    handoff once we switch to a soaring mode there.
    """
    hl, ho = hotspot
    items = [
        # seq, frame, command, p1..p4, lat, lon, alt
        (0, mavutil.mavlink.MAV_FRAME_GLOBAL, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
         0, 0, 0, 0, 0.0, 0.0, 0.0),
        (1, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
         mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 15, 0, 0, 0, 0.0, 0.0, takeoff_alt),
        (2, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
         mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, hl, ho, cruise_alt),
        (3, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
         mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM, 0, 0, 0, 0, hl, ho, cruise_alt),
    ]
    m.mav.mission_count_send(m.target_system, m.target_component,
                             len(items), mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    sent = set()
    deadline = time.time() + 30
    while len(sent) < len(items) and time.time() < deadline:
        req = m.recv_match(type=["MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"],
                           blocking=True, timeout=5)
        if req is None:
            continue
        if req.get_type() == "MISSION_ACK":
            break
        it = items[req.seq]
        seq, frame, cmd, p1, p2, p3, p4, lat, lon, alt = it
        m.mav.mission_item_int_send(
            m.target_system, m.target_component, seq, frame, cmd, 0, 1,
            p1, p2, p3, p4, int(lat * 1e7), int(lon * 1e7), alt,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        sent.add(req.seq)
    ack = m.recv_match(type="MISSION_ACK", blocking=True, timeout=10)
    return ack is not None and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED


def upload_goto_mission(m, hotspot, cruise_alt):
    """Re-target while already airborne: home, NAV_WAYPOINT@hotspot,
    NAV_LOITER_UNLIM@hotspot (no takeoff). Used for cross-country hops after the
    first. Caller should set mode AUTO and current waypoint to 1."""
    hl, ho = hotspot
    items = [
        (0, mavutil.mavlink.MAV_FRAME_GLOBAL, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
         0, 0, 0, 0, 0.0, 0.0, 0.0),
        (1, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
         mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, hl, ho, cruise_alt),
        (2, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
         mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM, 0, 0, 0, 0, hl, ho, cruise_alt),
    ]
    m.mav.mission_count_send(m.target_system, m.target_component,
                             len(items), mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    sent = set()
    deadline = time.time() + 30
    while len(sent) < len(items) and time.time() < deadline:
        req = m.recv_match(type=["MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"],
                           blocking=True, timeout=5)
        if req is None:
            continue
        if req.get_type() == "MISSION_ACK":
            break
        seq, frame, cmd, p1, p2, p3, p4, lat, lon, alt = items[req.seq]
        m.mav.mission_item_int_send(
            m.target_system, m.target_component, seq, frame, cmd, 0, 1,
            p1, p2, p3, p4, int(lat * 1e7), int(lon * 1e7), alt,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        sent.add(req.seq)
    ack = m.recv_match(type="MISSION_ACK", blocking=True, timeout=10)
    return ack is not None and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED


def set_current_wp(m, seq):
    m.mav.mission_set_current_send(m.target_system, m.target_component, seq)


def upload_qgc_file(m, path):
    """Upload a QGC WPL 110 .waypoints file (as written by planner/) verbatim."""
    items = []
    with open(path) as f:
        if not f.readline().startswith("QGC WPL"):
            raise ValueError("not a QGC WPL file")
        for line in f:
            line = line.strip()
            if not line:
                continue
            c = line.split("\t")
            items.append(dict(seq=int(c[0]), frame=int(c[2]), cmd=int(c[3]),
                              p1=float(c[4]), p2=float(c[5]), p3=float(c[6]), p4=float(c[7]),
                              x=float(c[8]), y=float(c[9]), z=float(c[10])))
    m.mav.mission_count_send(m.target_system, m.target_component,
                             len(items), mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    sent = set()
    deadline = time.time() + 30
    while len(sent) < len(items) and time.time() < deadline:
        req = m.recv_match(type=["MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"],
                           blocking=True, timeout=5)
        if req is None:
            continue
        if req.get_type() == "MISSION_ACK":
            break
        it = items[req.seq]
        m.mav.mission_item_int_send(
            m.target_system, m.target_component, it["seq"], it["frame"], it["cmd"], 0, 1,
            it["p1"], it["p2"], it["p3"], it["p4"],
            int(it["x"] * 1e7), int(it["y"] * 1e7), it["z"],
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        sent.add(req.seq)
    ack = m.recv_match(type="MISSION_ACK", blocking=True, timeout=10)
    return ack is not None and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED, len(items)


def goto_global(m, lat, lon, alt_rel, want_ack=False):
    """Command a GUIDED target position via MAV_CMD_DO_REPOSITION.

    This is the canonical "fly to here" for ArduPlane GUIDED (what a GCS sends);
    a raw SET_POSITION_TARGET_GLOBAL_INT is ignored by ArduPlane GUIDED, which
    then just loiters at wherever it entered GUIDED.

    If want_ack, wait for and return the COMMAND_ACK.result (None on timeout)."""
    m.mav.command_int_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_REPOSITION,
        0, 0,
        -1,   # p1: ground speed (-1 = default)
        0,    # p2: bitmask
        0,    # p3: reserved / loiter radius (0 = default)
        float("nan"),  # p4: yaw (NaN = unchanged)
        int(lat * 1e7), int(lon * 1e7), alt_rel)
    if not want_ack:
        return None
    ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
    return ack.result if ack and ack.command == mavutil.mavlink.MAV_CMD_DO_REPOSITION else None


def vehicle_position(m, timeout=2):
    """Return (lat, lon, rel_alt_m) or None."""
    msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=timeout)
    if msg is None:
        return None
    return msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0
