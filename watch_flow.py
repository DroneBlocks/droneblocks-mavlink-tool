#!/usr/bin/env python3
"""Watch optical-flow health over MAVLink (OPTICAL_FLOW_RAD, #106).

Zero firmware/DDS changes — just subscribes to the existing mavlink-router
link. Works the same on UP-T201 (DEXI-3) and ARK Flow (DEXI-5): both surface
in PX4 as sensor_optical_flow -> the OPTICAL_FLOW_RAD MAVLink message.

Leave it running and power-cycle the drone / swap sensors — it prints rate +
quality once a second, and shouts when nothing is arriving.

    ./venv/bin/python watch_flow.py <drone-ip>
"""
import sys
import time
from pymavlink import mavutil

ip = sys.argv[1] if len(sys.argv) > 1 else None
if not ip:
    sys.exit("usage: watch_flow.py <drone-ip>   (the DEXI you plugged the sensor into)")

# 14550 is the GCS endpoint (server mode) — PX4 won't stream until it sees a
# GCS heartbeat, so we send one. source_system 255 = ground station.
m = mavutil.mavlink_connection(f"udpout:{ip}:14550", source_system=255, source_component=190)
print(f"connecting to {ip}:14550 … waiting for autopilot heartbeat")

last_hb = 0.0
tgt_sys = tgt_comp = None
requested = False
count = 0
window_start = time.time()
last_flow = 0.0
last_quality = last_dist = None

while True:
    now = time.time()

    # 1 Hz GCS heartbeat keeps the stream alive
    if now - last_hb >= 1.0:
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                             mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        last_hb = now

    msg = m.recv_match(blocking=True, timeout=0.5)
    if msg:
        t = msg.get_type()
        if t == "HEARTBEAT" and tgt_sys is None:
            # On a mavlink-router fabric several things emit heartbeats (GCS,
            # router, mavlink2rest). Only the real autopilot has a valid
            # autopilot field (PX4=12); GCS/router report INVALID(8). Targeting
            # the wrong one means SET_MESSAGE_INTERVAL never reaches PX4 and you
            # get a false "no flow".
            if msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                tgt_sys, tgt_comp = msg.get_srcSystem(), msg.get_srcComponent()
                print(f"autopilot found: sys={tgt_sys} comp={tgt_comp} "
                      f"(autopilot={msg.autopilot}, type={msg.type})")
        if t == "OPTICAL_FLOW_RAD":
            count += 1
            last_flow = now
            last_quality = msg.quality            # 0-255; >~100 is usable
            last_dist = msg.distance              # paired rangefinder, m (-1 if none)

    # Ask PX4 to stream OPTICAL_FLOW_RAD at 10 Hz once we know the target
    if tgt_sys is not None and not requested:
        m.mav.command_long_send(
            tgt_sys, tgt_comp,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            mavutil.mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW_RAD, 100000, 0, 0, 0, 0, 0)
        requested = True

    # Once-a-second summary
    if now - window_start >= 1.0:
        rate = count / (now - window_start)
        silent = now - last_flow if last_flow else 999
        if rate > 0:
            print(f"FLOW  {rate:4.1f} Hz  quality={last_quality:>3}/255  "
                  f"dist={last_dist:.2f}m")
        elif silent > 3:
            why = "no autopilot link yet" if tgt_sys is None else \
                  "linked, but NO OPTICAL_FLOW_RAD — sensor not streaming"
            print(f"----  0.0 Hz  ({why})")
        count = 0
        window_start = now
