# -*- coding: utf-8 -*-
"""读取飞控关键参数，确认固件/机架配置"""
from pymavlink import mavutil
import time

master = mavutil.mavlink_connection("COM5", baud=115200, timeout=5)
hb = master.wait_heartbeat(timeout=15)
print(f"target: sys={master.target_system} comp={master.target_component}")

want = ["FRAME_CLASS", "FRAME_TYPE", "FORMAT_VERSION", "SERIAL1_PROTOCOL", "BRD_BOARD_ID"]
params = {}

for p in want:
    master.mav.param_request_read_send(master.target_system, master.target_component, p.encode(), -1)

end = time.time() + 5
while time.time() < end and len(params) < len(want):
    msg = master.recv_match(type="PARAM_VALUE", blocking=False)
    if msg:
        params[msg.param_id] = msg.param_value
    time.sleep(0.05)

for p in want:
    print(f"  {p} = {params.get(p, 'N/A')}")

master.close()
