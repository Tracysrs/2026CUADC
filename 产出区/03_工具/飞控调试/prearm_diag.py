# -*- coding: utf-8 -*-
"""解锁前诊断（阶段0）：只读汇总所有 PreArm 门禁状态，不发任何解锁指令。

汇总项：
  - 当前模式/已解锁标志（模式不合法或已解锁都会导致推杆无反应）
  - RC：链路 RSSI、油门是否在低位、失控保护状态
  - 电池电压 vs BATT_ARM_VOLT；传感器健康位
  - EKF 状态位（姿态/速度/位置是否收敛）
  - 罗盘一致性：内置 vs 外置(NEO-3) 地磁场模长差（ArduPilot 门限 ~150 mG）
  - 参数门限：ARMING_CHECK / BATT_ARM_VOLT / BRD_SAFETY_ENABLE / FS_THR_ENABLE / AHRS_GPS_USE
  - 期间收到的 STATUSTEXT（ArduPilot 周期性 PreArm 提示会出现在这里）

用法: python prearm_diag.py [COM口，默认 COM5]
"""

import math
import sys
import time

from pymavlink import mavutil

COPTER_MODES = {
    0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
    5: "LOITER", 6: "RTL", 7: "CIRCLE", 9: "LAND", 16: "POSHOLD",
    17: "BRAKE", 20: "GUIDED_NOGPS", 21: "SMART_RTL",
}
ARMABLE = {"STABILIZE", "ACRO", "ALT_HOLD", "LOITER", "POSHOLD", "GUIDED",
           "GUIDED_NOGPS", "SMART_RTL"}
WANT_PARAMS = ["ARMING_CHECK", "BATT_ARM_VOLT", "BRD_SAFETY_ENABLE",
               "BRD_SAFETY_DEFLT", "FS_THR_ENABLE", "AHRS_GPS_USE"]

# EKF_STATUS_REPORT 标志位（MAVLink 规范）
EKF_FLAGS = {0: "姿态", 1: "水平速度", 2: "垂直位置", 3: "水平相对位置",
             4: "水平绝对位置", 5: "AGL", 21: "常数位置漂移"}


def field_norm(msg):
    return math.sqrt(msg.xmag ** 2 + msg.ymag ** 2 + msg.zmag ** 2)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM5"
    m = mavutil.mavlink_connection(port, baud=115200, timeout=5)
    hb = m.wait_heartbeat(timeout=15)
    if hb is None:
        sys.exit("错误：未收到心跳")
    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    mode = COPTER_MODES.get(hb.custom_mode, f"未知({hb.custom_mode})")

    for mid, iv in [(mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, 100000),
                    (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 500000),
                    (mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 500000),
                    (mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU, 200000),
                    (mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU2, 200000)]:
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                0, mid, iv, 0, 0, 0, 0, 0)
    for p in WANT_PARAMS:
        m.mav.param_request_read_send(m.target_system, m.target_component,
                                      p.encode(), -1)

    st = []
    rc = sysekf = ekf = None
    imu1, imu2 = [], []
    params = {}
    deadline = time.time() + 10
    while time.time() < deadline:
        msg = m.recv_match(blocking=True, timeout=1.0)
        if msg is None:
            continue
        t = msg.get_type()
        if t == "STATUSTEXT":
            st.append((msg.severity, msg.text.rstrip("\x00")))
        elif t == "RC_CHANNELS":
            rc = msg
        elif t == "SYS_STATUS":
            sysekf = msg
        elif t == "EKF_STATUS_REPORT":
            ekf = msg
        elif t == "SCALED_IMU":
            imu1.append(field_norm(msg))
        elif t == "SCALED_IMU2":
            imu2.append(field_norm(msg))
        elif t == "PARAM_VALUE":
            key = msg.param_id.rstrip(b"\x00").decode() \
                if isinstance(msg.param_id, bytes) else msg.param_id
            params.setdefault(key, msg.param_value)
    m.close()

    print("\n=== 解锁前诊断（只读）===")
    print(f"模式: {mode} | 已解锁: {armed}")
    if mode not in ARMABLE and not armed:
        print(f"  ⛔ 模式 {mode} 不允许解锁 → 切 Stabilize/AltHold/Loiter/GUIDED")

    if rc:
        sbus_fs = getattr(rc, "sbus_failsafe", 0)
        print(f"RC:   RSSI {rc.rssi}/254 | 油门 ch3={rc.chan3_raw}us | "
              f"模式开关 ch5={rc.chan5_raw} | sbus_failsafe={sbus_fs}")
        if rc.chan3_raw > 1050:
            print("  ⛔ 油门不在低位 → 解锁杆要求油门最低（<1050us）")
        if sbus_fs:
            print("  ⛔ 接收机失控保护状态（遥控器/链路问题）")
        if rc.rssi == 0:
            print("  ⛔ 无 RC 信号（接收机未对频/线未插）")
    else:
        print("RC:   无数据 ⛔（遥控器没开或接收机没接好）")

    if sysekf:
        print(f"电池: {sysekf.voltage_battery / 1000:.2f}V | "
              f"传感器健康位: {sysekf.onboard_control_sensors_health:#010x}")
        arm_v = params.get("BATT_ARM_VOLT", 0)
        if arm_v and sysekf.voltage_battery / 1000 < arm_v:
            print(f"  ⛔ 电压低于 BATT_ARM_VOLT={arm_v:.2f}V")
    else:
        print("电池: 无数据")

    if ekf:
        on = [name for bit, name in EKF_FLAGS.items() if ekf.flags >> bit & 1]
        print(f"EKF:  flags={ekf.flags:#010x} 已收敛: {', '.join(on) or '无'}")
        if not (ekf.flags >> 0 & 1):
            print("  ⛔ EKF 姿态未收敛（刚上电等几秒）")
        if not (ekf.flags >> 4 & 1):
            print("  ⛔ EKF 无绝对水平位置（GPS 未定位到能解锁的程度）")
    else:
        print("EKF:  无数据")

    if imu1 and imu2:
        n1 = sorted(imu1)[len(imu1) // 2]
        n2 = sorted(imu2)[len(imu2) // 2]
        diff = abs(n1 - n2)
        print(f"罗盘: 内置 {n1:.0f} mG | 外置 {n2:.0f} mG | 模长差 {diff:.0f} mG")
        if diff > 150:
            print("  ⛔ 罗盘一致性超差（>150 mG）→ 校准罗盘/远离磁干扰")
        if not (80 < n1 < 850):
            print("  ⛔ 磁场模长超出合理范围（80~850 mG）→ 罗盘数据异常")
    else:
        print("罗盘: 数据不足（内置/外置至少一路无报文）")

    print("参数: " + " | ".join(f"{k}={v:g}" for k, v in sorted(params.items()))
          or "参数: 未取到")

    if st:
        print("\nSTATUSTEXT（近10s）:")
        for sev, text in st:
            print(f"  [{sev}] {text}")
    else:
        print("\nSTATUSTEXT: 10s 内无报文（无周期性 PreArm 报警；"
              "剩余门限只在解锁尝试瞬间报——地面站推解锁杆看消息页，"
              "或先过一遍上面标 ⛔ 的项）")
    print("\n提示：硬件安全开关状态 MAVLink 读不到——若以上全绿仍推杆无反应，"
          "连按两下安全开关再试。")


if __name__ == "__main__":
    main()
