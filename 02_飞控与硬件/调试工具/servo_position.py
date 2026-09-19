# -*- coding: utf-8 -*-
"""舵机定位（Windows 直连版）：把 AUX 舵机打到指定 PWM 并持续保持，供装盘/装爪相位定位

用法: python servo_position.py [COM口] [舵机列表] [PWM]   默认 COM5、SERVO9,10、1100(收拢)
  装舵机盘：打到 1100 收拢位并保持，舵机盘齿相位按「闭合锁死」装（1100=初始安全位，见 02 册）；
  释放位=1600（09-19 拍板）；打 1900=通道上限全开（⚠️ 瓶在位会真投放，先取瓶）。
保持期间每 5s 重发一次 DO_SET_SERVO 防丢失；Ctrl-C 或杀进程退出 = 断开会话，
安全开关恢复、PWM 停发，舵机脱力（盘已装好则无碍，下次会话/上电仍归 1100）。
判读同 servo_test.py：回读 SERVO_OUTPUT_RAW 实证飞控侧 PWM 真在走——
PWM 走了而舵机本体不动 = 舵机没电（UBEC 未上电/未注入 A 排母线，02 结构 §4 铁律1）。
⚠️ DO_SET_SERVO 不解锁、不转桨，但按台架纪律拆桨状态下操作。
"""
import sys
import time
from pymavlink import mavutil

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM5"
BAUD = 115200
SERVOS = (sorted({int(x) for x in sys.argv[2].split(",") if x.strip()})
          if len(sys.argv) > 2 else [9, 10])
PWM = int(sys.argv[3]) if len(sys.argv) > 3 else 1100
if any(s not in (9, 10, 11, 12, 13, 14) for s in SERVOS):
    print("ERROR: 舵机通道只支持 AUX 口 SERVO9~14（MAIN1~8 被电机/预留占用）")
    sys.exit(1)
if not 1000 <= PWM <= 2000:
    print("ERROR: PWM 须在 1000~2000")
    sys.exit(1)
CMD_DO_SET_SERVO = 183


def connect(port):
    print(f"[conn] trying {port} @ {BAUD} ...")
    try:
        m = mavutil.mavlink_connection(port, baud=BAUD, timeout=3)
        hb = m.wait_heartbeat(timeout=6)
    except Exception as e:
        print(f"[conn] {port} 不可用: {type(e).__name__}")
        return None, None
    if hb is None:
        m.close()
        return None, None
    return m, hb


def get_param(m, name, timeout=2.0):
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


def set_param_int(m, name, value):
    m.mav.param_set_send(m.target_system, m.target_component,
                         name.encode("utf-8"), float(value),
                         mavutil.mavlink.MAV_PARAM_TYPE_INT32)


def send_servo(m, ch):
    m.mav.command_long_send(m.target_system, m.target_component,
                            CMD_DO_SET_SERVO, 0, ch, PWM, 0, 0, 0, 0, 0)


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
    print("ERROR: 所有 COM 口都没有飞控心跳（MP 开着会占口，先断开 Mission Planner）")
    sys.exit(1)

print(f"[conn] OK  sys={master.target_system} comp={master.target_component}, "
      f"type={mavutil.mavlink.enums['MAV_TYPE'][hb.type].name}")

if hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
    print("ERROR: 飞控处于解锁(ARMED)状态，拒绝操作，请先上锁后重跑")
    master.close()
    sys.exit(1)
print("[safe] 飞控已上锁(DISARMED)\n")

master.mav.request_data_stream_send(master.target_system, master.target_component,
                                    mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
time.sleep(0.5)

# 核验① SERVOx_FUNCTION 必须=0（≠0 时功能占用会拦截 DO_SET_SERVO），现场自动改回
fixed = []
for s in SERVOS:
    v = get_param(master, f"SERVO{s}_FUNCTION")
    if v is None:
        print(f"ERROR: 读不到 SERVO{s}_FUNCTION，中止")
        master.close()
        sys.exit(1)
    if int(v) != 0:
        print(f"[check1] SERVO{s}_FUNCTION={int(v)}≠0 → 改回 0")
        set_param_int(master, f"SERVO{s}_FUNCTION", 0)
        time.sleep(0.5)
        if int(get_param(master, f"SERVO{s}_FUNCTION") or -1) != 0:
            print(f"ERROR: SERVO{s}_FUNCTION 改 0 失败，中止")
            master.close()
            sys.exit(1)
        fixed.append(s)
    print(f"[check1] SERVO{s}: FUNCTION=0 ✓  MIN={int(get_param(master, f'SERVO{s}_MIN') or 0)}"
          f" MAX={int(get_param(master, f'SERVO{s}_MAX') or 0)}")
if fixed:
    print(f"[check1] 已现场改 FUNCTION=0 的通道: {fixed}（测完可用 params-diff 核对）\n")

# 核验② 解开硬件安全开关（闭合时输出口全静默，09-09 日志 §五 的坑；仅本会话生效）
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_SAFETY_SWITCH_STATE, 0,
    1, 0, 0, 0, 0, 0, 0)
ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
if ack is not None and ack.result == 0:
    print("[check2] 硬件安全开关已解除（仅本会话生效）")
else:
    print("[check2] !! 安全开关解除未确认——若 PWM 回读不动，先怀疑这个")

print(f"\n[plan] 通道 {SERVOS} → 定位 {PWM}µs（{'收拢' if PWM == 1100 else '释放' if PWM == 1600 else '自定义'}），"
      "3 秒后开始，Ctrl-C 取消")
try:
    for i in range(3, 0, -1):
        print(f"    {i} ...")
        time.sleep(1)
except KeyboardInterrupt:
    print("\n已取消，一个指令都没发")
    master.close()
    sys.exit(0)

# 首发定位 + 回读实证
ok_all = True
for s in SERVOS:
    send_servo(master, s)
    master.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    time.sleep(0.3)
    raw = None
    deadline = time.time() + 2.0
    while time.time() < deadline and raw is None:
        msg = master.recv_match(type="SERVO_OUTPUT_RAW", blocking=False)
        if msg is not None:
            raw = getattr(msg, f"servo{s}_raw", None)
        time.sleep(0.02)
    hit = raw is not None and abs(raw - PWM) <= 25
    ok_all = ok_all and hit
    print(f"  S{s}: 定位 {PWM}µs  回读={raw if raw is not None else '无数据'}"
          f"{' ✓' if hit else ' ← 飞控输出未跟上!'}")

if not ok_all:
    print("\n有通道回读未跟上，先重跑；仍复现则查 FUNCTION/安全开关（见上方核验输出）。")
    master.close()
    sys.exit(1)

print(f"\n[hold] 舵机已到 {PWM}µs 并持续保持中——现在可以装舵机盘/装爪。")
print("       PWM 走了而舵机本体不动 = 舵机没电：查 UBEC 是否上电输出 6.0V 并注入 A 排。")
print("       保持期间每 5s 重发一次；Ctrl-C 或结束进程即停（盘装好后停无碍）。")
try:
    n = 0
    while True:
        time.sleep(5)
        n += 1
        for s in SERVOS:
            send_servo(master, s)
        master.recv_match(type="COMMAND_ACK", blocking=True, timeout=1)
        print(f"  [hold +{n * 5:>3}s] 重发 {'/'.join(f'S{s}={PWM}' for s in SERVOS)} ✓", flush=True)
except KeyboardInterrupt:
    print("\n[hold] 手动结束，保持已停")
master.close()
