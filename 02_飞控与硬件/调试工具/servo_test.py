# -*- coding: utf-8 -*-
"""舵机投放测试（Windows 直连版）：COM 口 MAVLink 直发 DO_SET_SERVO 跑投放时序

用法: python servo_test.py [COM口] [舵机列表]   默认 COM5、SERVO9；测两路: servo_test.py COM5 9,10

对应 Jetson 侧的 fc_servo_test.sh（ROS2/mavros 版）——逻辑一致：
  前提核验 → 解会话级安全开关 → 1500→1100(收拢)→1900(释放)→…→1500 时序。
判读：每步回读 SERVO_OUTPUT_RAW 实证飞控侧 PWM 真在走——
  PWM 走了而舵机本体不动 = 舵机没电（输出母线悬空，需 UBEC 5V 注入并共地，02 结构模块 §4 铁律1）。
⚠️ DO_SET_SERVO 不解锁、不转桨，但按台架纪律拆桨状态下操作。
"""
import sys
import time
from pymavlink import mavutil

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM5"
BAUD = 115200
SERVOS = (sorted({int(x) for x in sys.argv[2].split(",") if x.strip()})
          if len(sys.argv) > 2 else [9])
if any(s not in (9, 10, 11, 12, 13, 14) for s in SERVOS):
    print("ERROR: 舵机通道只支持 AUX 口 SERVO9~14（MAIN1~8 被电机/预留占用）")
    sys.exit(1)
LOOP = len(sys.argv) > 3 and sys.argv[3] == "loop"  # 连续收拢/释放，台架边扫边查线

STOW = 1100     # 与 mission_params.yaml servo_stowed_pwm 一致
RELEASE = 1900  # 与 servo_release_pwm 一致
CMD_DO_SET_SERVO = 183

# 时序与 Jetson 版 fc_servo_test.sh 完全同源：(PWM, 停留秒, 说明)
SEQ = [(1500, 2.0, "中位"),
       (STOW, 2.0, "收拢"),
       (RELEASE, 1.2, "释放(投放!)"),
       (STOW, 2.0, "回仓"),
       (RELEASE, 1.2, "复投"),
       (STOW, 1.5, "回仓"),
       (1500, 1.0, "归中")]


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


def read_servo_raw(m, ch, timeout=2.0):
    """回读 SERVO_OUTPUT_RAW 指定通道当前 PWM（拿不到返回 None）"""
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = m.recv_match(type="SERVO_OUTPUT_RAW", blocking=False)
        if msg is not None:
            return getattr(msg, f"servo{ch}_raw", None)
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
    print("ERROR: 所有 COM 口都没有飞控心跳（MP 开着会占口，先断开 Mission Planner）")
    sys.exit(1)

print(f"[conn] OK  sys={master.target_system} comp={master.target_component}, "
      f"type={mavutil.mavlink.enums['MAV_TYPE'][hb.type].name}")

if hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
    print("ERROR: 飞控处于解锁(ARMED)状态，拒绝测试，请先上锁后重跑")
    sys.exit(1)
print("[safe] 飞控已上锁(DISARMED)\n")

# SERVO_OUTPUT_RAW 不在默认串口流里，显式要一份 ALL@10Hz 用于逐步回读
master.mav.request_data_stream_send(master.target_system, master.target_component,
                                    mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
time.sleep(0.5)

# ── 核验① SERVOx_FUNCTION 必须=0（≠0 时功能占用会拦截 DO_SET_SERVO）──
fixed = []
for s in SERVOS:
    v = get_param(master, f"SERVO{s}_FUNCTION")
    if v is None:
        print(f"ERROR: 读不到 SERVO{s}_FUNCTION，中止")
        sys.exit(1)
    if int(v) != 0:
        print(f"[check1] SERVO{s}_FUNCTION={int(v)}≠0 → 改回 0")
        set_param_int(master, f"SERVO{s}_FUNCTION", 0)
        time.sleep(0.5)
        if int(get_param(master, f"SERVO{s}_FUNCTION") or -1) != 0:
            print(f"ERROR: SERVO{s}_FUNCTION 改 0 失败，中止")
            sys.exit(1)
        fixed.append(s)
    vmin = get_param(master, f"SERVO{s}_MIN")
    vmax = get_param(master, f"SERVO{s}_MAX")
    print(f"[check1] SERVO{s}: FUNCTION=0 ✓  MIN={int(vmin) if vmin else '?'} "
          f"MAX={int(vmax) if vmax else '?'}（{STOW}/{RELEASE} 应落在区间内）")
if fixed:
    print(f"[check1] 已现场改 FUNCTION=0 的通道: {fixed}（参数文件（现 02_飞控与硬件/）里本就是 0，测完可 params-diff 核对）\n")

# ── 核验② 解开硬件安全开关（闭合时输出口全静默，09-09 日志 §五 的坑）──
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_SAFETY_SWITCH_STATE, 0,
    1, 0, 0, 0, 0, 0, 0)  # param1=1 解除（仅本会话生效）
ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
if ack is not None and ack.result == 0:
    print("[check2] 硬件安全开关已解除（仅本会话生效）")
else:
    print("[check2] !! 安全开关解除未确认——若 PWM 回读始终不动，先怀疑这个")

print(f"\n[plan] 通道 {SERVOS}，时序 {'→'.join(str(p[0]) for p in SEQ)}；"
      "3 秒后开始，Ctrl-C 取消")
try:
    for i in range(3, 0, -1):
        print(f"    {i} ...")
        time.sleep(1)
except KeyboardInterrupt:
    print("\n已取消，一个指令都没发")
    master.close()
    sys.exit(0)

if LOOP:
    print(f"\n[loop] {SERVOS} 连续 收拢/释放 20 轮（约 1 分钟，Ctrl-C 随时停）")
    print("  期间可：万用表量舵机插头 +/− 电压、换线、换口——保持动作在打，改动立见")
    try:
        for cyc in range(1, 21):
            for s in SERVOS:
                for pwm, label in ((STOW, "收拢"), (RELEASE, "释放")):
                    master.mav.command_long_send(
                        master.target_system, master.target_component,
                        CMD_DO_SET_SERVO, 0, s, pwm, 0, 0, 0, 0, 0)
                    master.recv_match(type="COMMAND_ACK", blocking=True, timeout=1)
                    print(f"  轮{cyc:02d} S{s}: {label} {pwm}µs", flush=True)
                    time.sleep(1.2)
    except KeyboardInterrupt:
        pass
    for s in SERVOS:
        master.mav.command_long_send(
            master.target_system, master.target_component,
            CMD_DO_SET_SERVO, 0, s, STOW, 0, 0, 0, 0, 0)
        master.recv_match(type="COMMAND_ACK", blocking=True, timeout=1)
    print("\n[loop] 结束，已归收拢位")
    master.close()
    sys.exit(0)

ok_all = True
for s in SERVOS:
    print(f"\n===== SERVO{s}（AUX{s - 8}） =====")
    for pwm, hold, label in SEQ:
        master.mav.command_long_send(
            master.target_system, master.target_component,
            CMD_DO_SET_SERVO, 0, s, pwm, 0, 0, 0, 0, 0)
        ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
        res = ack.result if ack is not None else -1
        time.sleep(0.3)  # 等输出刷新后回读
        raw = read_servo_raw(master, s)
        hit = raw is not None and abs(raw - pwm) <= 25
        if res != 0 or not hit:
            ok_all = False
        print(f"  {label:<10} {pwm}µs  ACK={'ACCEPTED' if res == 0 else res}  "
              f"回读={raw if raw is not None else '无数据'}"
              f"{' ✓' if hit else ' ← 飞控输出未跟上!'}")
        time.sleep(hold)

master.close()
print("\n===== 判读 =====")
if ok_all:
    print("  飞控侧链路全通：每步 DO_SET_SERVO 均 ACCEPTED 且 SERVO_OUTPUT_RAW 跟上。")
    print("  若舵机本体没动 → 问题在舵机电气侧，依次查：")
    print("    ① 输出母线 5V（UBEC 注入 A 排 +/−，飞控不供电——02 结构 §4 铁律1）")
    print("    ② 共地（UBEC 地与飞控地通）  ③ 信号针对位（S/+/− 三针）")
    print("  若动了但方向/行程不对 → 对调 mission_params.yaml 的 stowed/release 或调 MIN/MAX。")
else:
    print("  有步骤 ACK≠ACCEPTED 或回读没跟上 → 先重跑；仍复现则查 FUNCTION/安全开关（见上方核验输出）。")
