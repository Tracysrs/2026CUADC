# -*- coding: utf-8 -*-
"""复原解锁检查：ARMING_SKIPCHK 恢复为 force_arm_skipgps.py 记录的原值。

无记录文件时回退 1280（09-19/09-24 参数备份的飞控常设值，跳过 GPS/位置检查
的既定口径）；加 --zero 可清零为全检查启用（fc_armrun.sh 的复原口径）。
用法：python restore_arming.py [COM口] [--zero]
"""
import os
import sys
import time
from pymavlink import mavutil

STATE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_arming_skipchk_orig.txt")
DEFAULT_VALUE = 1280  # 飞控常设值（09-24 备份实证）

args = [a for a in sys.argv[1:] if not a.startswith("--")]
zero = "--zero" in sys.argv
if zero:
    value = 0
elif os.path.exists(STATE_FILE):
    with open(STATE_FILE) as f:
        value = int(f.read().strip())
else:
    value = DEFAULT_VALUE
    print(f"无记录文件，回退常设值 {value}（--zero 可改清零）")

PORT = args[0] if args else "COM17"
BAUD = 115200

print(f"Connecting to {PORT} @ {BAUD} ...")
master = mavutil.mavlink_connection(PORT, baud=BAUD, timeout=5)
if master.wait_heartbeat(timeout=15) is None:
    print("ERROR: no heartbeat received")
    sys.exit(1)
print("Heartbeat OK")


def param_get(name, timeout=8):
    master.mav.param_request_read_send(master.target_system, master.target_component, name.encode(), -1)
    end = time.time() + timeout
    while time.time() < end:
        msg = master.recv_match(type="PARAM_VALUE", blocking=False)
        if msg is not None and msg.param_id.strip("\x00") == name:
            return msg.param_value
        time.sleep(0.05)
    return None


cur = param_get("ARMING_SKIPCHK")
if cur is None:
    print("ERROR: 读不到 ARMING_SKIPCHK，中止")
    master.close()
    sys.exit(1)
print(f"当前 ARMING_SKIPCHK = {int(cur)}，目标 {value}")

if int(cur) == value:
    print("已是目标值，无需改动")
else:
    for _ in range(3):
        master.mav.param_set_send(master.target_system, master.target_component,
                                  b"ARMING_SKIPCHK", float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        cur = param_get("ARMING_SKIPCHK")
        if cur is not None and int(cur) == value:
            break
        time.sleep(1)
    else:
        print("ERROR: 写入未生效（回读不符），请重试或检查连接")
        master.close()
        sys.exit(1)

print(f"✅ ARMING_SKIPCHK = {value}")
if value == 0:
    print("全部解锁检查已启用（GPS/位置门禁恢复）")
else:
    print("GPS/位置检查仍为跳过态（常设口径）——户外 GPS 飞行前建议 --zero 清零")
if os.path.exists(STATE_FILE):
    os.remove(STATE_FILE)
    print("记录文件已清除")
master.close()
