#!/usr/bin/env python3
"""GCS 心跳泵：让 ArduPilot 在 mavros 链路上开始流送遥测。

背景：ArduPilot 只在 mavlink 通道上看到 GCS 心跳后才按 SRx_ 速率流送
数据报文，而 mavros 不会冒充 GCS → 裸 SITL + mavros 时 odom/vfr_hud/
compass 全部无数据（心跳/STATUSTEXT 除外，它们不受流控）。

原理：mavros 以 gcs_url 开一个桥（推荐 tcp-l://0.0.0.0:14550），本脚本作为
"GCS" 接入并持续发 GCS 心跳；mavros 把桥上流量转发到 FCU 链路，
ArduPilot 检测到 GCS 即开始流送，数据经同一条链路回到 mavros。

用法：python3 gcs_pump.py（配合 m1_sitl_accept.sh 自动启停）
关键点：必须【先发后收】——UDP/TCP 对端地址要在发出首包后才确立。
"""

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
