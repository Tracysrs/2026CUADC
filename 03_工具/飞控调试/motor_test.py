# -*- coding: utf-8 -*-
"""电机顺序/方向测试：M1→M4 逐个转 1 秒（务必拆桨状态下使用）

用法: python motor_test.py [COM口]
默认 COM5@115200，连不上则自动扫描其他 COM 口。
"""
import sys
import time
from pymavlink import mavutil

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM5"
BAUD = 115200
THROTTLE = 25.0   # 测试油门 %（高于 MOT_SPIN_MIN=15%）
RUN_S = 1.0       # 每个电机转动时长
GAP_S = 2.5       # 电机之间间隔，便于观察记录

# ArduPilot MAV_CMD_DO_MOTOR_TEST 的 param1 是"测试序列号"（AP_MotorsMatrix.cpp
# 的 _test_order），不是输出口编号。Quad X 序列=从右前顺时针：M1(前右)→M4(后右)
# →M2(后左)→M3(前左)。此表把输出口换成对应序列号，实现按输出口 M1→M4 测试。
OUT2SEQ = {1: 1, 2: 3, 3: 4, 4: 2}
POS = {1: "前右", 2: "后左", 3: "前左", 4: "后右"}

# 用法: python motor_test.py [COM口] [输出列表]  如: python motor_test.py COM5 3 或 COM5 1,3
TEST_OUTPUTS = [int(x) for x in sys.argv[2].split(",")] if len(sys.argv) > 2 else [1, 2, 3, 4]

ACK_NAMES = {0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED",
             3: "UNSUPPORTED", 4: "FAILED", 5: "IN_PROGRESS"}


def connect(port):
    print(f"[conn] trying {port} @ {BAUD} ...")
    try:
        m = mavutil.mavlink_connection(port, baud=BAUD, timeout=3)
        hb = m.wait_heartbeat(timeout=6)
    except Exception as e:
        # 口不存在/被占用（MP 开着、USB 拔掉等）在这里抛异常，必须吞掉，
        # 否则主流程走不到下面的自动扫描分支（2026-09-09 踩坑）
        print(f"[conn] {port} 不可用: {type(e).__name__}")
        return None, None
    if hb is None:
        m.close()
        return None, None
    return m, hb


master, hb = connect(PORT)
if master is None:
    import serial.tools.list_ports
    for p in sorted(serial.tools.list_ports.comports()):
        if p.device == PORT:
            continue
        master, hb = connect(p.device)
        if master:
            break
if master is None:
    print("ERROR: 所有 COM 口都没有飞控心跳")
    sys.exit(1)

print(f"[conn] OK  port ok, sys={master.target_system} comp={master.target_component}, "
      f"type={mavutil.mavlink.enums['MAV_TYPE'][hb.type].name}")

if hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
    print("ERROR: 飞控处于解锁(ARMED)状态，拒绝电机测试，请先上锁后重跑")
    sys.exit(1)
print("[safe] 飞控已上锁(DISARMED)，开始测试\n")


def drain_statustext(seconds):
    end = time.time() + seconds
    while time.time() < end:
        msg = master.recv_match(type="STATUSTEXT", blocking=False)
        if msg is None:
            time.sleep(0.05)
            continue
        print(f"    [FC] {msg.text}")


# 清掉历史 STATUSTEXT
master.recv_match(type="STATUSTEXT", blocking=False)

results = {}
for out in TEST_OUTPUTS:
    print(f"===== 输出 M{out}（{POS[out]}）：{THROTTLE:.0f}% 油门，转 {RUN_S:.0f} 秒 =====")
    sys.stdout.flush()
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, 0,
        OUT2SEQ[out],    # param1 测试序列号（换算自输出口）
        0,               # param2 油门类型 = 百分比
        THROTTLE,        # param3 油门值
        RUN_S,           # param4 持续秒数
        1,               # param5 测试电机数
        0, 0, 0)
    ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    if ack is None:
        print(f"    !! M{out} 无 ACK（命令未送达），跳过")
        results[out] = "NO_ACK"
        continue
    res = ACK_NAMES.get(ack.result, str(ack.result))
    print(f"    MAV_CMD_DO_MOTOR_TEST(seq={OUT2SEQ[out]}) -> {res}")
    results[out] = res
    if ack.result == 0:
        drain_statustext(RUN_S + GAP_S)  # 转动 1s + 间隔 2.5s
    else:
        drain_statustext(2.0)  # 收集飞控给出的拒绝原因
        print("    飞控拒绝了电机测试，停止后续电机")
        break

master.close()
print("\n===== 汇总 =====")
for i in TEST_OUTPUTS:
    print(f"  M{i}（{POS[i]}）: {results.get(i, '未执行')}")
print("\n请记录每个电机旋转方向（从电机正上方往下看：顺时针 CW / 逆时针 CCW）")
