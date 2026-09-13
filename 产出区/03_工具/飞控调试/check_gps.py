# -*- coding: utf-8 -*-
"""检查 GPS/罗盘接入状态：定位、卫星数、罗盘检测、EKF"""
import time
from pymavlink import mavutil

master = mavutil.mavlink_connection("COM5", baud=115200, timeout=5)
hb = master.wait_heartbeat(timeout=15)
print(f"已连接，载具: {mavutil.mavlink.enums['MAV_TYPE'][hb.type].name}")

# 请求数据流
for msg_id, iv in [(mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 500000),
                   (mavutil.mavlink.MAVLINK_MSG_ID_GPS2_RAW, 500000),
                   (mavutil.mavlink.MAVLINK_MSG_ID_SYSTEM_TIME, 1000000)]:
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                 msg_id, iv, 0, 0, 0, 0, 0)

# 读关键参数
want = ["GPS_TYPE", "GPS_TYPE2", "GPS_AUTO_CONFIG", "SERIAL3_PROTOCOL", "SERIAL4_PROTOCOL",
        "COMPASS_DEV_ID", "COMPASS_DEV_ID2", "COMPASS_DEV_ID3", "COMPASS_PRIO1_ID",
        "COMPASS_PRIO2_ID", "COMPASS_PRIO3_ID", "CAN_P1_DRIVER", "GPS_DRIVER_VERSION" ]
params = {}
for p in want:
    master.mav.param_request_read_send(master.target_system, master.target_component, p.encode(), -1)

end = time.time() + 6
last_gps = None
last_gps2 = None
while time.time() < end:
    msg = master.recv_match(blocking=False)
    if msg:
        t = msg.get_type()
        if t == "PARAM_VALUE":
            key = msg.param_id.rstrip(b"\x00").decode() if isinstance(msg.param_id, bytes) else msg.param_id
            if key not in params:
                params[key] = msg.param_value
        elif t == "GPS_RAW_INT":
            last_gps = msg
        elif t == "GPS2_RAW":
            last_gps2 = msg
    time.sleep(0.02)

FIX = ["无GPS", "无定位", "2D", "3D", "DGPS", "RTK浮点", "RTK固定", "静态", "PPP"]
print("\n--- GPS 配置参数 ---")
for k in want:
    if k in params:
        print(f"  {k} = {params[k]}")

if last_gps:
    print("\n--- GPS1 ---")
    ft = last_gps.fix_type
    print(f"  定位: {FIX[ft] if ft < len(FIX) else ft} | 卫星: {last_gps.satellites_visible} | HDOP: {last_gps.eph/100:.2f}")
    print(f"  纬度: {last_gps.lat/1e7:.7f} 经度: {last_gps.lon/1e7:.7f} 海拔: {last_gps.alt/1000:.1f}m")
    vdop = getattr(last_gps, "vdop", None) or getattr(last_gps, "epv", 0)
    print(f"  VDOP: {vdop/100:.2f} | 速度: {last_gps.vel/100:.2f}m/s")
else:
    print("\n--- GPS1: 无数据 ---")

if last_gps2:
    ft = last_gps2.fix_type
    print(f"\n--- GPS2: {FIX[ft] if ft < len(FIX) else ft} | 卫星: {last_gps2.satellites_visible} ---")

master.close()
