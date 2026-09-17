# -*- coding: utf-8 -*-
"""RTK rover（瑞杰 RTK-01-R，接 V6X GPS2 口）一键配置+验证。

背景（2026-09-13 实测）：本版 4.7-beta 固件没有 GPS2_BAUD 参数，波特率挂在
SERIAL4_BAUD（旧式枚举：57=57600、230=230400、921=921600）。rover 出厂
NMEA @921600，故需 GPS2_TYPE=3(NMEA) + SERIAL4_BAUD=921 + 重启。
GPS_AUTO_SWITCH 保持 0（只看不切，SSOT 定位决策：NEO-3 为主）。

用法: python rtk_setup.py [COM口，默认 COM5]
  - 检查并写入参数（已正确则跳过）→ 可选重启 → 盯 GPS2 60s 出 RTK 状态
注意：基站 RTK-01-B 必须上电完成 survey-in（LORA 灯亮），否则 rover 永远
只有单点解，不会出现 RTK浮点(5)/RTK固定(6)。
"""

import sys
import time

from pymavlink import mavutil

FIX_NAMES = ["无GPS", "无定位", "2D", "3D", "DGPS",
             "RTK浮点", "RTK固定", "静态", "PPP"]
WANT = {"GPS2_TYPE": 3, "SERIAL4_BAUD": 921, "GPS_AUTO_SWITCH": 0}


def get_param(m, name, wait=3.0):
    m.mav.param_request_read_send(m.target_system, m.target_component,
                                  name.encode(), -1)
    end = time.time() + wait
    while time.time() < end:
        r = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if r is not None:
            pid = r.param_id.rstrip("\x00") if isinstance(r.param_id, str) \
                else r.param_id.rstrip(b"\x00").decode()
            if pid == name:
                return r.param_value
    return None


def set_param(m, name, value, wait=4.0):
    m.mav.param_set_send(m.target_system, m.target_component, name.encode(),
                         float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    end = time.time() + wait
    while time.time() < end:
        r = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if r is not None:
            pid = r.param_id.rstrip("\x00") if isinstance(r.param_id, str) \
                else r.param_id.rstrip(b"\x00").decode()
            if pid == name:
                return r.param_value
    return None


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM5"
    only_g2 = len(sys.argv) > 2 and sys.argv[2] == "2"  # 只盯 GPS2（GPS1 未接时不刷屏）
    m = mavutil.mavlink_connection(port, baud=115200, timeout=5)
    m.wait_heartbeat(timeout=15)
    print("已连接飞控")

    changed = False
    for name, want in WANT.items():
        cur = get_param(m, name)
        if cur is None:
            print(f"{name}: 读不到（参数不存在？）")
            continue
        if int(cur) == want:
            print(f"{name} = {cur:g} 已正确")
        else:
            got = set_param(m, name, want)
            print(f"{name}: {cur:g} -> {want}，回读 {got if got is not None else '超时'}")
            # SERIALx_BAUD/GPS_TYPE 均需重启才生效——只要动过笔就重启，
            # 不能因"回读已正确"而跳过（2026-09-14 实测：写完不重启 rover 依旧无数据）
            changed = True

    if changed:
        print("\n参数有变动，重启飞控生效...")
        time.sleep(1)
        m.mav.command_long_send(m.target_system, m.target_component,
                                # 注意枚举全名是 *_REBOOT_SHUTDOWN（2026-09-14 实锤：
                                # 旧写法 MAV_CMD_PREFLIGHT_REBOOT 不存在，重启从未生效）
                                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                                0, 1, 0, 0, 0, 0, 0, 0)
        m.close()
        print("等 25s 待飞控重启...")
        time.sleep(25)
        m = None
        for i in range(5):
            try:
                m = mavutil.mavlink_connection(port, baud=115200, timeout=5)
                if m.wait_heartbeat(timeout=15) is not None:
                    break
            except Exception as e:
                print(f"重连第{i + 1}次失败: {e}")
                m = None
                time.sleep(5)
        if m is None:
            sys.exit("飞控未恢复，检查 USB")

    # 验证：盯 GPS1/GPS2 60s
    print("\n盯 GPS1/GPS2 60s（GPS2 需 rover 接线正常；RTK 解还需基站 survey-in 完成）")
    # GPS_RAW_INT=24、GPS2_RAW=124（旧写法 116 是别的报文，GPS2 从未被请求过）
    for mid, iv in [(24, 1000000), (124, 1000000),
                    (mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU, 1000000),
                    (mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU2, 1000000),
                    (mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU3, 1000000)]:
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                0, mid, iv, 0, 0, 0, 0, 0)
    last = {}
    g2_fix = 0
    end = time.time() + 60
    while time.time() < end:
        msg = m.recv_match(blocking=True, timeout=1.0)
        if msg is None:
            continue
        t = msg.get_type()
        if t in ("GPS_RAW_INT", "GPS2_RAW"):
            if only_g2 and t == "GPS_RAW_INT":
                last[t] = "未接（本轮只盯 GPS2）"
                continue
            ft = msg.fix_type
            if t == "GPS2_RAW":
                g2_fix = ft
            last[t] = (f"{FIX_NAMES[ft] if ft < len(FIX_NAMES) else ft} "
                       f"星={msg.satellites_visible} HDOP={msg.eph / 100:.2f}")
            rem = int(end - time.time())
            tag = "GPS1" if t == "GPS_RAW_INT" else "GPS2"
            print(f"  [{rem:3d}s] {tag}: {last[t]}", flush=True)
        elif t.startswith("SCALED_IMU"):
            last[t] = "有报文"
    m.close()

    print("\n=== 结论 ===")
    g2 = last.get("GPS2_RAW")
    print(f"GPS1(NEO-3): {last.get('GPS_RAW_INT', '无数据')}")
    print(f"GPS2(RTK rover): {g2 or '全程无报文'}")
    if g2 and "RTK" in g2:
        print("✅ RTK 链路全通")
    elif g2_fix >= 1:
        print("⚠ rover 通信正常但未 RTK 解 → 检查基站是否上电且 survey-in 完成"
              "（LORA 灯亮），Fix 收敛要等")
    else:
        # fix=0(NO_GPS)=整个监视期没解析到一条有效 NMEA；只要收到有效语句，
        # 哪怕无定位也是 fix=1（2026-09-16 实测：旧文案此时打"通信正常"，误导排查方向）
        print("❌ FC 没收到 rover 任何有效 NMEA（fix=0）→ 物理链路按序排查：")
        print("   1. rover 三颗 LED 是否亮（电源/LORA/RTK）")
        print("   2. 6P GH 线针序：FC k 针 → rover k+1 针循环移位"
              "（09-15 勘误定稿；旧直通接法=5V 撞 GND）")
        print("   3. 是否插在飞控 GPS2 口（SERIAL4）；NEO-3 必须留在 GPS1")
        print("   4. rover 出厂波特率若被改过，用厂家 APP 改回 921600")


if __name__ == "__main__":
    main()
