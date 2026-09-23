# -*- coding: utf-8 -*-
"""检查 CAN/DroneCAN 接入状态：CAN 参数、GPS 实例、罗盘、SLCAN 总线嗅探（COM16）"""
import sys
import time
from pymavlink import mavutil
import serial.tools.list_ports
import serial as pyserial

def find_port(name):
    for p in serial.tools.list_ports.comports():
        if name in (p.description or ""):
            return p.device
    return None

# --- MAVLink 侧：参数 + GPS 实例 ---
port = find_port("ArduPilot MAVLink")
if not port:
    print("找不到飞控 MAVLink 口（ArduPilot MAVLink）——查地面站是否占口")
    sys.exit(1)
m = mavutil.mavlink_connection(port, baud=115200, timeout=3)
m.wait_heartbeat(timeout=10)
print(f"已连接 {port}")

want = ["CAN_P1_DRIVER", "CAN_D1_PROTOCOL", "CAN_P1_BITRATE",
        "GPS1_TYPE", "GPS2_TYPE", "GPS_AUTO_SWITCH", "SERIAL8_PROTOCOL",
        "COMPASS_DEV_ID", "COMPASS_DEV_ID2", "COMPASS_DEV_ID3", "COMPASS_DEV_ID4",
        "COMPASS_PRIO1_ID", "COMPASS_PRIO2_ID", "COMPASS_PRIO3_ID"]
for p in want:
    m.mav.param_request_read_send(m.target_system, m.target_component, p.encode(), -1)
for msg_id, iv in [(24, 200000), (124, 200000)]:
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                            msg_id, iv, 0, 0, 0, 0, 0)
params, gps1, gps2 = {}, None, None
end = time.time() + 8
while time.time() < end:
    msg = m.recv_match(blocking=False)
    if msg is not None:
        t = msg.get_type()
        if t == "PARAM_VALUE":
            k = msg.param_id.rstrip(b"\x00").decode() if isinstance(msg.param_id, bytes) else msg.param_id
            params.setdefault(k, msg.param_value)
        elif t == "GPS_RAW_INT":
            gps1 = msg
        elif t == "GPS2_RAW":
            gps2 = msg
    time.sleep(0.02)
m.close()

print("\n--- CAN 与 GPS 参数 ---")
for k in want:
    if k in params:
        print(f"  {k} = {params[k]:g}")
    else:
        print(f"  {k} = 不存在")

FIX = ["无GPS", "无定位", "2D", "3D", "DGPS", "RTK浮点", "RTK固定", "静态", "PPP"]
print("\n--- GPS 实例 ---")
for name, g in [("GPS1(DroneCAN)", gps1), ("GPS2(串口)", gps2)]:
    if g:
        ft = g.fix_type
        print(f"  {name}: {FIX[ft] if ft < len(FIX) else ft} | 卫星 {g.satellites_visible}")
    else:
        print(f"  {name}: 无数据")

# --- SLCAN 侧：总线嗅探（listen-only，飞控驱动负责 ACK）---
print("\n--- SLCAN 总线嗅探（15s, listen-only）---")
slcan_port = find_port("ArduPilot SLCAN")
if not slcan_port:
    print("  找不到 SLCAN 口（ArduPilot SLCAN）——查 SERIAL8_PROTOCOL=13 是否已落机")
elif "CAN_P1_DRIVER" in params and params["CAN_P1_DRIVER"] != 1:
    print(f"  CAN_P1_DRIVER={params['CAN_P1_DRIVER']:g} ≠ 1——先落 CAN 驱动钥匙再嗅探")
else:
    try:
        s = pyserial.Serial(slcan_port, baudrate=115200, timeout=0.5)
        s.write(b"\r"); time.sleep(0.2); s.reset_input_buffer()
        s.write(b"L\r")
        frames = []
        end = time.time() + 15
        while time.time() < end:
            line = s.readline().strip()
            if line:
                frames.append(line.decode(errors="replace"))
        s.write(b"C\r"); time.sleep(0.1); s.close()
        print(f"  共 {len(frames)} 帧")
        for f in frames[:8]:
            print(f"    {f}")
        if frames:
            ids = {}
            for f in frames:
                if f and f[0] == "T":
                    nid = int(f[1:9], 16) >> 8
                    ids[f"节点{nid}"] = ids.get(f"节点{nid}", 0) + 1
            print(f"  按 DroneCAN 源节点聚合: {ids}")
        else:
            print("  总线静默——查：①模块 LED（供电）②CAN1 5V/GND ③H/L 是否对调 ④模块 CAN 模式（LGC，疑点 CAN1_FD_EN_MODE=2=FD 模式）")
    except Exception as e:
        print(f"  SLCAN 打开失败: {e}")

# --- 结论 ---
print("\n--- 结论 ---")
drv = params.get("CAN_P1_DRIVER", -1)
proto = params.get("CAN_D1_PROTOCOL", -1)
if drv == 1 and proto == 1:
    print("  FC 侧 CAN 驱动+DroneCAN 已就位（CAN_P1_DRIVER=1, CAN_D1_PROTOCOL=1, GPS1_TYPE=9）")
    if gps1 is not None and gps1.fix_type >= 2:
        print(f"  DroneCAN GPS 有定位（{FIX[gps1.fix_type]}）——CAN 路线已通")
    elif frames:
        print("  总线有帧但 GPS 无定位——节点在，Fix 没出（查模块 GNSS 模式/天线）")
    else:
        print("  总线静默——FC 侧已就绪，卡点在模块侧/接线")
else:
    print(f"  FC 侧 CAN 未就位（DRIVER={drv:g}, PROTOCOL={proto:g}）——需落 CAN 钥匙参数")
