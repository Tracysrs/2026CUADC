# -*- coding: utf-8 -*-
"""连接飞控 COM5，读取基本信息（版本、类型、状态、传感器）"""
import sys
import time
from pymavlink import mavutil

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM5"   # COM 号漂移时命令行指定
BAUD = 115200

print(f"Connecting to {PORT} @ {BAUD} ...")
master = mavutil.mavlink_connection(PORT, baud=BAUD, timeout=5)

# 等待第一个心跳
heartbeat = master.wait_heartbeat(timeout=15)
if heartbeat is None:
    print("ERROR: no heartbeat received")
    sys.exit(1)

print(f"Heartbeat OK:")
print(f"  system_id={master.target_system}, component_id={master.target_component}")
print(f"  type={mavutil.mavlink.enums['MAV_TYPE'][heartbeat.type].name}")
print(f"  autopilot={mavutil.mavlink.enums['MAV_AUTOPILOT'][heartbeat.autopilot].name}")
print(f"  base_mode={heartbeat.base_mode}, custom_mode={heartbeat.custom_mode}")
print(f"  system_status={mavutil.mavlink.enums['MAV_STATE'][heartbeat.system_status].name}")

# 请求 AUTOPILOT_VERSION
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, 0, 1,
    0, 0, 0, 0, 0, 0)

# 请求关键数据流
for msg_id, interval in [
    (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 200000),
    (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 500000),
    (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 500000),
    (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 500000),
]:
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, msg_id, interval,
        0, 0, 0, 0, 0)

# 收集 4 秒数据
version = None
modes_seen = {}
end = time.time() + 4.0
last_att = None
last_sys = None
last_pos = None
last_gps = None

while time.time() < end:
    msg = master.recv_match(blocking=False)
    if msg is None:
        time.sleep(0.05)
        continue
    t = msg.get_type()
    if t == "AUTOPILOT_VERSION":
        version = msg
    elif t == "ATTITUDE":
        last_att = msg
    elif t == "SYS_STATUS":
        last_sys = msg
    elif t == "GLOBAL_POSITION_INT":
        last_pos = msg
    elif t == "GPS_RAW_INT":
        last_gps = msg

def fmt_sw(v):
    if not v:
        return "unknown"
    return f"{v >> 16 & 0xFF}.{v >> 8 & 0xFF}.{v & 0xFF}"

if version:
    print("\n--- Autopilot Version ---")
    print(f"  flight_sw: {fmt_sw(version.flight_sw_version)}")
    print(f"  flight_sw git: 0x{version.flight_sw_version:08X}")
    print(f"  middleware_sw: {fmt_sw(version.middleware_sw_version)}")
    print(f"  board_version: 0x{version.board_version:08X}")
    try:
        print(f"  flight_custom_version: {bytes(version.flight_custom_version).decode('ascii', 'replace')}")
    except Exception:
        pass
    print(f"  os_custom_version: {bytes(version.os_custom_version).decode('ascii', 'replace') if version.os_custom_version else 'n/a'}")

# ArduPilot 飞行模式
MODE_MAP = {0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER",
            6: "RTL", 7: "CIRCLE", 9: "LAND", 11: "DRIFT", 13: "SPORT", 14: "FLIP",
            15: "AUTOTUNE", 16: "POSHOLD", 17: "BRAKE", 18: "THROW", 19: "AVOID_ADSB",
            20: "GUIDED_NOGPS", 21: "SMART_RTL", 22: "FLOWHOLD", 23: "FOLLOW", 24: "ZIGZAG",
            25: "SYSTEMID", 26: "AUTOROTATE", 27: "AUTO_RTL"}
if heartbeat.custom_mode in MODE_MAP:
    print(f"\nCurrent flight mode: {MODE_MAP[heartbeat.custom_mode]}")

if last_sys:
    print("\n--- SYS_STATUS ---")
    print(f"  battery voltage: {last_sys.voltage_battery / 1000:.2f} V")
    print(f"  battery remaining: {last_sys.battery_remaining}%")
    print(f"  sensors healthy bitmask: 0x{last_sys.onboard_control_sensors_health:08X}")

if last_gps:
    print("\n--- GPS ---")
    fix_names = ["NO GPS", "NO FIX", "2D FIX", "3D FIX", "DGPS", "RTK Float", "RTK Fixed", "STATIC"]
    print(f"  fix_type: {fix_names[last_gps.fix_type] if last_gps.fix_type < len(fix_names) else last_gps.fix_type}")
    print(f"  satellites: {last_gps.satellites_visible}")
    print(f"  lat: {last_gps.lat / 1e7}, lon: {last_gps.lon / 1e7}, alt: {last_gps.alt / 1000} m")

if last_pos:
    print("\n--- Position ---")
    print(f"  relative_alt: {last_pos.relative_alt / 1000:.2f} m")

if last_att:
    import math
    print("\n--- Attitude ---")
    print(f"  roll: {math.degrees(last_att.roll):.2f} deg")
    print(f"  pitch: {math.degrees(last_att.pitch):.2f} deg")
    print(f"  yaw: {math.degrees(last_att.yaw):.2f} deg")

master.close()
print("\nDone.")
