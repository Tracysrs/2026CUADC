#!/usr/bin/env python3
"""GCS 心跳泵（先发后收）：让 ArduPilot 在 mavros 链路开始流送"""
import time
from pymavlink import mavutil
conn = mavutil.mavlink_connection('tcp:127.0.0.1:14550',
                                  source_system=255, source_component=190)
def hb():
    conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
print('pump: send-first, waiting FCU heartbeat...', flush=True)
deadline = time.time() + 60
while time.time() < deadline:
    hb()
    msg = conn.recv_match(blocking=True, timeout=1.0)
    if msg is not None and msg.get_type() == 'HEARTBEAT':
        break
print('pump: link alive, request streams', flush=True)
conn.mav.request_data_stream_send(1, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
n = 0
while True:
    hb()
    time.sleep(0.5)
    n += 1
    if n % 20 == 0:
        conn.mav.request_data_stream_send(1, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
