# -*- coding: utf-8 -*-
"""强制解锁前置：临时跳过 GPS/位置类解锁检查（ARMING_SKIPCHK=1280）并记录原值。

只动 GPS 相关检查，不碰罗盘/EKF/电池/RC 等其余门禁。1280 为 09-24 室内首飞
实证值（4.7-beta 位掩码逐位核对挂账，值本身经真机验证）。
⚠️ 跳过后 EKF 无绝对位置：GUIDED/LOITER/RTL 等 GPS 模式不可用，仅限 STABILIZE
   室内手动或无桨台架；户外飞行前必须跑 restore_arming.py 复原。
原值写入同目录 _arming_skipchk_orig.txt，restore_arming.py 读取它复原。
用法：python force_arm_skipgps.py [COM口]
"""
import os
import sys
import time
from pymavlink import mavutil

SKIP_VALUE = 1280  # 跳过 GPS/位置类解锁检查（09-24 实证）
STATE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_arming_skipchk_orig.txt")

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM17"
BAUD = 115200

print(f"Connecting to {PORT} @ {BAUD} ...")
master = mavutil.mavlink_connection(PORT, baud=BAUD, timeout=5)
if master.wait_heartbeat(timeout=15) is None:
    print("ERROR: no heartbeat received")
    sys.exit(1)
print("Heartbeat OK")


def param_get(name, timeout=8):
    """读单个参数（PARAM_REQUEST_READ → PARAM_VALUE）"""
    master.mav.param_request_read_send(master.target_system, master.target_component, name.encode(), -1)
    end = time.time() + timeout
    while time.time() < end:
        msg = master.recv_match(type="PARAM_VALUE", blocking=False)
        if msg is not None and msg.param_id.strip("\x00") == name:
            return msg.param_value
        time.sleep(0.05)
    return None


def param_set(name, value):
    """写参数并回读确认（含 4.7-beta 懒加载可能的延迟，重试 3 次）"""
    for _ in range(3):
        master.mav.param_set_send(master.target_system, master.target_component,
                                  name.encode(), float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        got = param_get(name)
        if got is not None and abs(got - value) < 0.5:
            return True
        time.sleep(1)
    return False


orig = param_get("ARMING_SKIPCHK")
if orig is None:
    print("ERROR: 读不到 ARMING_SKIPCHK，中止（未改动任何参数）")
    master.close()
    sys.exit(1)
orig_i = int(orig)
print(f"原 ARMING_SKIPCHK = {orig_i}")

if orig_i == SKIP_VALUE:
    print("已是 1280（GPS/位置检查本就跳过），无需改动")
    master.close()
    sys.exit(0)

with open(STATE_FILE, "w") as f:
    f.write(str(orig_i))

if not param_set("ARMING_SKIPCHK", SKIP_VALUE):
    print("ERROR: 写入未生效（回读不符），请重试或检查连接")
    master.close()
    sys.exit(1)

print(f"✅ ARMING_SKIPCHK = {SKIP_VALUE}（原值 {orig_i} 已存 {os.path.basename(STATE_FILE)}）")
print("现在可用 prearm_diag.py 复查剩余门禁，再解锁。")
print("⚠️ 用完务必跑：python restore_arming.py")
master.close()
