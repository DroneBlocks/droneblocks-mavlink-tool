"""
Indoor-flight telemetry streamer for PX4 via MAVLink.

Subscribes to ESTIMATOR_STATUS, LOCAL_POSITION_NED, VIBRATION,
DISTANCE_SENSOR, and OPTICAL_FLOW_RAD, merges them into a single
health sample at ~2 Hz, and invokes a user-supplied callback.

Designed to be driven by either a CLI (print sink) or a FastAPI
WebSocket endpoint (broadcast sink) — the subscription logic is
transport-agnostic.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, asdict
from typing import Callable, Optional
from pymavlink import mavutil

# ESTIMATOR_STATUS_FLAGS bitmask values (MAVLink common)
FLAG_ATTITUDE             = 1 << 0
FLAG_VELOCITY_HORIZ       = 1 << 1
FLAG_VELOCITY_VERT        = 1 << 2
FLAG_POS_HORIZ_REL        = 1 << 3
FLAG_POS_HORIZ_ABS        = 1 << 4
FLAG_POS_VERT_ABS         = 1 << 5
FLAG_POS_VERT_AGL         = 1 << 6
FLAG_CONST_POS_MODE       = 1 << 7
FLAG_PRED_POS_HORIZ_REL   = 1 << 8
FLAG_PRED_POS_HORIZ_ABS   = 1 << 9
FLAG_GPS_GLITCH           = 1 << 10
FLAG_ACCEL_ERROR          = 1 << 11


@dataclass
class HealthSample:
    t: float                         # seconds, monotonic
    const_pos_mode: bool             # TRUE = no horizontal aiding (BAD in flight)
    pred_pos_horiz_rel: bool         # TRUE = position prediction valid
    pos_horiz_rel: bool              # TRUE = relative position being fused
    eph: float                       # horizontal accuracy (m)
    pos_x: float                     # NED (m)
    pos_y: float
    pos_z: float
    vx: float                        # NED velocity (m/s)
    vy: float
    vz: float
    flow_q: Optional[int]            # 0..255 optical flow quality
    rng_m: Optional[float]           # rangefinder distance (m)
    vib_x: float                     # vibration RMS per axis (m/s^2)
    vib_y: float
    vib_z: float
    clip_0: int                      # IMU clipping events
    clip_1: int
    clip_2: int

    def to_dict(self) -> dict:
        return asdict(self)


# Messages we care about + their MAVLink IDs and requested intervals (us)
_SUBSCRIPTIONS = [
    (mavutil.mavlink.MAVLINK_MSG_ID_ESTIMATOR_STATUS,   100_000),  # 10 Hz
    (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 100_000),
    (mavutil.mavlink.MAVLINK_MSG_ID_VIBRATION,          200_000),  #  5 Hz
    (mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR,    100_000),
    (mavutil.mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW_RAD,   100_000),
]


def _request_streams(conn) -> None:
    """Ask the FC to stream our messages at the rates we want."""
    for msg_id, interval_us in _SUBSCRIPTIONS:
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            msg_id, interval_us, 0, 0, 0, 0, 0,
        )


def stream_health(
    host: str = "192.168.68.61",
    port: int = 14550,
    on_sample: Optional[Callable[[HealthSample], None]] = None,
    stop_flag: Optional[Callable[[], bool]] = None,
    emit_hz: float = 2.0,
    on_connect: Optional[Callable[[], None]] = None,
) -> None:
    """
    Connect to PX4 via udpout:<host>:<port> (mavlink-router GCS endpoint),
    stream health signals, and invoke on_sample(HealthSample) at ~emit_hz.

    Stops when stop_flag() returns True (polled between messages).
    """
    conn = mavutil.mavlink_connection(f"udpout:{host}:{port}")
    _send_heartbeat(conn)
    conn.wait_heartbeat(timeout=10)
    _request_streams(conn)
    if on_connect:
        on_connect()

    latest = {"es": None, "lp": None, "vib": None, "dist": None, "flow": None}
    last_emit = 0.0
    last_hb = 0.0
    emit_period = 1.0 / emit_hz

    while stop_flag is None or not stop_flag():
        now = time.monotonic()

        # Keep mavlink-router's GCS endpoint from dropping us
        if now - last_hb > 1.0:
            _send_heartbeat(conn)
            last_hb = now

        msg = conn.recv_match(blocking=True, timeout=0.1)
        if msg is not None:
            mtype = msg.get_type()
            if   mtype == "ESTIMATOR_STATUS":   latest["es"]   = msg
            elif mtype == "LOCAL_POSITION_NED": latest["lp"]   = msg
            elif mtype == "VIBRATION":          latest["vib"]  = msg
            elif mtype == "DISTANCE_SENSOR":    latest["dist"] = msg
            elif mtype == "OPTICAL_FLOW_RAD":   latest["flow"] = msg

        if now - last_emit >= emit_period and on_sample is not None:
            last_emit = now
            on_sample(_build_sample(now, latest))


def _build_sample(t: float, m: dict) -> HealthSample:
    es, lp, vib, dist, flow = m["es"], m["lp"], m["vib"], m["dist"], m["flow"]
    flags = es.flags if es else 0
    return HealthSample(
        t=t,
        const_pos_mode    = bool(flags & FLAG_CONST_POS_MODE),
        pred_pos_horiz_rel= bool(flags & FLAG_PRED_POS_HORIZ_REL),
        pos_horiz_rel     = bool(flags & FLAG_POS_HORIZ_REL),
        eph   = es.pos_horiz_accuracy if es else float("nan"),
        pos_x = lp.x  if lp else float("nan"),
        pos_y = lp.y  if lp else float("nan"),
        pos_z = lp.z  if lp else float("nan"),
        vx    = lp.vx if lp else float("nan"),
        vy    = lp.vy if lp else float("nan"),
        vz    = lp.vz if lp else float("nan"),
        flow_q = flow.quality if flow else None,
        rng_m  = (dist.current_distance / 100.0) if dist else None,
        vib_x  = vib.vibration_x if vib else 0.0,
        vib_y  = vib.vibration_y if vib else 0.0,
        vib_z  = vib.vibration_z if vib else 0.0,
        clip_0 = vib.clipping_0  if vib else 0,
        clip_1 = vib.clipping_1  if vib else 0,
        clip_2 = vib.clipping_2  if vib else 0,
    )


def _send_heartbeat(conn) -> None:
    conn.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0,
    )
