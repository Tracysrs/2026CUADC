# -*- coding: utf-8 -*-
"""电机顺序测试：严格按输出口 M1→M2→M3→M4 逐个发 1 秒信号（务必拆桨状态下使用）

用法: python motor_test.py [COM口] [输出列表]
默认 COM5@115200，连不上则自动扫描其他 COM 口。

"严格"靠跑前三道核验保证（任一不过就拒绝测试，宁可不动也不乱动）：
① 读机架参数取输出口→测试序列号换算表（DO_MOTOR_TEST param1 是测试序列号
   不是输出口，直接发 1,2,3,4 会按顺时针点亮 M1,M4,M2,M3，见 2026-09-09 日志 §三）；
② 读 SERVO1~4_FUNCTION 确认输出口 M1~M4 真的绑定 Motor1~4；
③ 代发 DO_SET_SAFETY_SWITCH_STATE=1 解开硬件安全开关——安全开关闭合时
   输出口全静默、电调收不到任何信号（见 2026-09-09 日志 §五）。
"""
import sys
import time
from pymavlink import mavutil

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM5"
BAUD = 115200
THROTTLE = 25.0   # 测试油门 %（高于 MOT_SPIN_MIN=15%）
RUN_S = 1.0       # 每个电机转动时长
GAP_S = 2.5       # 电机之间间隔，便于观察记录
COUNTDOWN_S = 3   # 开跑前倒计时，期间 Ctrl-C 取消

# MAV_CMD_DO_MOTOR_TEST 的 param1 是"测试序列号"（AP_MotorsMatrix 的
# _test_order），不是输出口编号，按机架查表把输出口换成序列号。
# Quad X 序列=从右前顺时针：M1(前右)→M4(后右)→M2(后左)→M3(前左)。
FRAME_LAYOUTS = {
    (1, 1): {  # FRAME_CLASS=1 四轴 + FRAME_TYPE=1 X 型
        "name": "Quad X",
        "seq": {1: 1, 2: 3, 3: 4, 4: 2},
        "pos": {1: "前右", 2: "后左", 3: "前左", 4: "后右"},
    },
    (1, 0): {  # FRAME_TYPE=0 十字（+）型
        "name": "Quad +",
        "seq": {1: 1, 2: 2, 3: 3, 4: 4},
        "pos": {1: "正前", 2: "正右", 3: "正左", 4: "正后"},
    },
}

ACK_NAMES = {0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED",
             3: "UNSUPPORTED", 4: "FAILED", 5: "IN_PROGRESS"}

# 用法: python motor_test.py [COM口] [输出列表]  如: python motor_test.py COM5 3 或 COM5 1,3
# 无论怎么传，强制升序去重——保证严格按 M1→M2→M3→M4 的顺序发信号
TEST_OUTPUTS = (sorted({int(x) for x in sys.argv[2].split(",") if x.strip()})
                if len(sys.argv) > 2 else [1, 2, 3, 4])
if any(o not in (1, 2, 3, 4) for o in TEST_OUTPUTS):
    print("ERROR: 输出口编号只支持 1~4")
    sys.exit(1)


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


def get_param(m, name, timeout=2.0):
    """单发 PARAM_REQUEST_READ 并等回 PARAM_VALUE，拿不到返回 None"""
    for _ in range(2):
        m.mav.param_request_read_send(m.target_system, m.target_component,
                                      name.encode("utf-8"), -1)
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = m.recv_match(type="PARAM_VALUE", blocking=False)
            if msg is not None:
                pid = msg.param_id
                if isinstance(pid, bytes):
                    pid = pid.decode("utf-8", "ignore")
                if pid.split("\x00")[0] == name:
                    return msg.param_value
            time.sleep(0.02)
    return None


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
print("[safe] 飞控已上锁(DISARMED)\n")

# ── 核验① 机架 → 输出口→序列号 换算表 ──
frame_class = get_param(master, "FRAME_CLASS")
frame_type = get_param(master, "FRAME_TYPE")
layout = None
if frame_class is not None and frame_type is not None:
    layout = FRAME_LAYOUTS.get((int(frame_class), int(frame_type)))
if layout is None:
    print(f"ERROR: 机架 FRAME_CLASS={frame_class}/FRAME_TYPE={frame_type} "
          "不在换算表内（只内置四轴 +/X），请先补 FRAME_LAYOUTS 再测，拒绝盲发")
    sys.exit(1)
OUT2SEQ = layout["seq"]
POS = layout["pos"]
print(f"[check1] 机架 {layout['name']}(CLASS={int(frame_class)},TYPE={int(frame_type)})，"
      f"输出口→序列号换算 {OUT2SEQ}")

# ── 核验② 输出口 M1~M4 必须绑定 Motor1~4（SERVOx_FUNCTION=33~36）──
# DO_MOTOR_TEST 按"电机"寻址：绑定若不对，"按序测试"只是假象，点亮的是别的口
mism = []
for out in (1, 2, 3, 4):
    v = get_param(master, f"SERVO{out}_FUNCTION")
    if v is None or int(v) != 32 + out:
        mism.append((out, v))
if mism:
    for out, v in mism:
        print(f"ERROR: SERVO{out}_FUNCTION={v}，期望 {32 + out}(Motor{out})——"
              "输出口绑定与假设不符，拒绝测试")
    sys.exit(1)
print("[check2] SERVO1~4_FUNCTION=Motor1~4，输出口 M1~M4 绑定确认")

# ── 核验③ 解开硬件安全开关（否则输出口静默，电调收不到信号）──
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_SAFETY_SWITCH_STATE, 0,
    1, 0, 0, 0, 0, 0, 0)  # param1=1 打开安全开关（DANGEROUS，仅限拆桨台架）
ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
if ack is not None and ack.result == 0:
    print("[check3] 硬件安全开关已解除（仅本会话生效）")
else:
    res = ACK_NAMES.get(ack.result, "无ACK") if ack is not None else "无ACK"
    print(f"[check3] !! 安全开关解除未确认({res})——若测试全程电调静默，"
          "就是 09-09 日志 §五 的坑：安全开关双闸下 PWM 口不出信号")


def drain_statustext(seconds):
    end = time.time() + seconds
    while time.time() < end:
        msg = master.recv_match(type="STATUSTEXT", blocking=False)
        if msg is None:
            time.sleep(0.05)
            continue
        print(f"    [FC] {msg.text}")


plan = "→".join(f"M{o}({POS[o]})" for o in TEST_OUTPUTS)
print(f"\n[plan] 严格按 {plan} 顺序，每台 {THROTTLE:.0f}%×{RUN_S:.0f}s、"
      f"间隔 {GAP_S:.1f}s；{COUNTDOWN_S} 秒后开始，Ctrl-C 取消")
try:
    for i in range(COUNTDOWN_S, 0, -1):
        print(f"    {i} ...")
        time.sleep(1)
except KeyboardInterrupt:
    print("\n已取消，一个测试信号都没发")
    master.close()
    sys.exit(0)

# 清掉历史 STATUSTEXT
master.recv_match(type="STATUSTEXT", blocking=False)

results = {}
for out in TEST_OUTPUTS:
    print(f"\n===== 输出 M{out}（{POS[out]}）：{THROTTLE:.0f}% 油门，转 {RUN_S:.0f} 秒 =====")
    sys.stdout.flush()
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, 0,
        OUT2SEQ[out],    # param1 测试序列号（核验①换算自输出口）
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
        drain_statustext(RUN_S + GAP_S)  # 等满 转动+间隔 再碰下一台，保证严格串行
    else:
        drain_statustext(2.0)  # 收集飞控给出的拒绝原因
        print("    飞控拒绝了电机测试，立即停止（宁缺勿乱，不跳号继续）")
        break

master.close()
print("\n===== 汇总 =====")
for i in TEST_OUTPUTS:
    print(f"  M{i}（{POS[i]}）: {results.get(i, '未执行')}")
print("\n请记录每个电机旋转方向（从电机正上方往下看：顺时针 CW / 逆时针 CCW）")
