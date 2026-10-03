#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Jetson↔飞控 MAVLink 连通性一键自检（单文件，可直接单独拷到 Jetson 运行）

用法:
  python3 fc_link_test.py                       # 默认 /dev/ttyTHS1 @ 921600（FC TELEM3，与 bench_m2_real.sh 同源）
  python3 fc_link_test.py /dev/ttyACM0 115200   # 换串口/波特率
  python3 fc_link_test.py udp:0.0.0.0:14550     # 任意 pymavlink 连接串（第二参波特率被忽略）

检查项: 串口占用探测 / 心跳 / 固件版本 / 链路RTT(PING×5) / 参数点名×4 / 消息流速率(默认8s) / GPS·姿态概况
判据: 硬项(心跳≥0.7Hz + 固件版本可读 + ATTITUDE≥4Hz)全过 → PASS 退出0；任一失败 → FAIL 退出1
注意: 串口独占——先停 mavros 再跑（脚本会探测占用并警告，但不会替你停）
依赖: pip3 install pymavlink
"""
import os
import statistics
import subprocess
import sys
import time

DEFAULT_DEV = "/dev/ttyTHS1"
DEFAULT_BAUD = 921600
OBS_SECONDS = 8          # 消息流速率观测窗
HB_TIMEOUT = 10          # 心跳等待上限(秒)

PARAM_SPOT = [  # (名称, 期望值, 说明) —— 只读不改；缺参数只告警不判FAIL（残表/懒加载组历史）
    ("FRAME_CLASS", 1, "期望1=Quad（曾漂成4=OctaQuad）"),
    ("GPS1_TYPE", 9, "期望9=DroneCAN（NEO3Pro@CAN1主GPS）"),
    ("BATT_LOW_VOLT", 10.5, "电池失效保护低压阈值"),
    ("FLTMODE6", 6, "CH6飞行模式期望6=RTL"),
]

results = []  # (名称, 级别, 是否OK, 详情)  级别: HARD/WARN/INFO


def record(name, level, ok, detail):
    results.append((name, level, ok, detail))
    mark = {"HARD": "✓" if ok else "✗", "WARN": "!" if ok else "!", "INFO": "·"}[level]
    print(f"  [{mark}] {name}: {detail}")


def warn_if_busy(dev):
    """Linux 串口被占（典型：mavros 在跑）只警告不退出——占用者会分走数据导致速率虚低"""
    if not (os.name == "posix" and dev.startswith("/dev/")):
        return
    if not os.path.exists(dev):
        sys.exit(f"ERROR: 串口设备不存在: {dev}（查接线/设备树/是否 ttyUSB*）")
    try:
        r = subprocess.run(["fuser", dev], capture_output=True, text=True, timeout=5)
        pids = r.stdout.split()
        if pids:
            print(f"[!!] 警告: {dev} 已被占用 PID={pids}，测试结果会失真。先停占用者:")
            for pid in pids:
                subprocess.run(["ps", "-o", "pid=,cmd=", "-p", pid])
            print("     例如: pkill -f mavros && 重跑本脚本")
    except FileNotFoundError:
        pass  # 无 fuser 命令则跳过探测
    except Exception:
        pass


def read_param(master, name, timeout=2.0):
    """点名读单参数，超时返回 None（rtk_watch.py 同款回执匹配写法）"""
    master.mav.param_request_read_send(
        master.target_system, master.target_component, name.encode("ascii"), -1)
    end = time.time() + timeout
    while time.time() < end:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg is None:
            continue
        pid = msg.param_id.rstrip(b"\x00").decode() if isinstance(msg.param_id, bytes) else msg.param_id
        if pid.strip() == name:
            return msg.param_value
    return None


def main():
    conn = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_DEV
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_BAUD
    obs = float(sys.argv[3]) if len(sys.argv) > 3 else OBS_SECONDS

    try:
        from pymavlink import mavutil
    except ImportError:
        sys.exit("ERROR: 缺 pymavlink →  pip3 install pymavlink")

    print(f"== Jetson↔飞控连通性自检 ==  连接={conn}  波特率={baud}  观测窗={obs:.0f}s")
    warn_if_busy(conn)

    print("\n-- 1. 心跳 --")
    try:
        master = mavutil.mavlink_connection(conn, baud=baud, timeout=5)
    except Exception as e:
        sys.exit(f"ERROR: 打不开 {conn}: {e}\n"
                 f"  → 查设备是否存在/拼写（ls /dev/tty*）、权限(dialout)、是否被占用")
    t0 = time.time()
    hb = master.wait_heartbeat(timeout=HB_TIMEOUT)
    if hb is None:
        record("心跳", "HARD", False, f"{HB_TIMEOUT}s 内无心跳")
        print("\n排查清单:")
        print("  1) FC 是否上电（插电有无开机音/LED）")
        print("  2) 是否接在别的电脑上（本机 Windows 端 Mission Planner / COM 口要断开）")
        print(f"  3) TELEM3 三线 TX/RX/GND 是否在位（FC 8/10/6 针），波特率是否 {DEFAULT_BAUD}")
        print("  4) 串口权限: sudo usermod -aG dialout $USER 后重新登录")
        print("  5) 换连接串试: python3 fc_link_test.py /dev/ttyUSB0 115200")
        sys.exit(1)
    src = hb.get_srcSystem()
    print(f"  来源 sysid={src} compid={hb.get_srcComponent()}（首帧耗时 {time.time()-t0:.1f}s）")
    record("心跳", "HARD", True,
           f"sysid={src} type={mavutil.mavlink.enums['MAV_TYPE'][hb.type].name} "
           f"status={mavutil.mavlink.enums['MAV_STATE'][hb.system_status].name}")
    ts, tc = master.target_system, master.target_component

    print("\n-- 2. 固件版本 --")
    master.mav.command_long_send(ts, tc,
        mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, 0, 1, 0, 0, 0, 0, 0, 0)
    ver = None
    end = time.time() + 3
    while time.time() < end and ver is None:
        m = master.recv_match(type="AUTOPILOT_VERSION", blocking=True, timeout=0.5)
        if m:
            ver = m
    if ver is None:
        record("固件版本", "HARD", False, "3s 内无 AUTOPILOT_VERSION 回包")
    else:
        sw = ver.flight_sw_version
        cust = bytes(ver.flight_custom_version).decode("ascii", "replace")
        record("固件版本", "HARD", True,
               f"{sw>>16&0xFF}.{sw>>8&0xFF}.{sw&0xFF} (git {cust}) board=0x{ver.board_version:08X}")

    print("\n-- 3. 链路RTT (PING×5) --")
    rtts = []
    for seq in range(5):
        master.mav.ping_send(int(time.time() * 1e6) & 0xFFFFFFFF, seq, ts, tc)
        sent_at = time.time()
        p = master.recv_match(type="PING", blocking=True, timeout=1.0)
        while p is not None and p.seq != seq:
            p = master.recv_match(type="PING", blocking=True, timeout=0.5)
        if p is not None:
            rtts.append((time.time() - sent_at) * 1000)
        time.sleep(0.2)
    record("PING回包", "INFO", bool(rtts),
           (f"{len(rtts)}/5 回包, RTT avg={statistics.mean(rtts):.1f}ms max={max(rtts):.1f}ms"
            if rtts else "0/5 回包（链路单向通也可能，看消息流）"))

    print("\n-- 4. 参数点名×4 (只读) --")
    for name, expect, note in PARAM_SPOT:
        v = read_param(master, name)
        if v is None:
            record(f"参数{name}", "WARN", False, "未读到（超时/该组懒加载未实例化）")
        elif abs(v - expect) < 1e-6:
            record(f"参数{name}", "WARN", True, f"={v:g} 符合预期（{note}）")
        else:
            record(f"参数{name}", "WARN", False, f"={v:g} ≠ 期望{expect:g}（{note}）")

    print(f"\n-- 5. 消息流速率 ({obs:.0f}s 观测) --")
    for mid, us, why in [(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 100000, "姿态10Hz"),
                         (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 500000, "系统状态2Hz"),
                         (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 500000, "位置2Hz")]:
        master.mav.command_long_send(ts, tc,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mid, us, 0, 0, 0, 0, 0)
    counts, last = {}, {}
    end = time.time() + obs
    while time.time() < end:
        m = master.recv_match(blocking=False)
        if m is None:
            time.sleep(0.02)
            continue
        t = m.get_type()
        if m.get_srcSystem() != ts:  # 只统计 FC 发来的
            continue
        counts[t] = counts.get(t, 0) + 1
        last[t] = m
    hb_rate = counts.get("HEARTBEAT", 0) / obs
    att_rate = counts.get("ATTITUDE", 0) / obs
    record("HEARTBEAT速率", "HARD", hb_rate >= 0.7, f"{hb_rate:.2f} Hz（判据≥0.7）")
    record("ATTITUDE速率", "HARD", att_rate >= 4.0,
           f"{att_rate:.1f} Hz（请求10Hz，判据≥4；若串口被占用会虚低）")
    for t, floor in [("SYS_STATUS", 0.5), ("GLOBAL_POSITION_INT", 0.5)]:
        r = counts.get(t, 0) / obs
        record(f"{t}速率", "WARN", r >= floor, f"{r:.2f} Hz（请求2Hz，判据≥{floor}）")
    print(f"  其他流量: " + (", ".join(f"{k}={v}" for k, v in sorted(counts.items())
          if k not in ("HEARTBEAT", "ATTITUDE", "SYS_STATUS", "GLOBAL_POSITION_INT")) or "无"))

    print("\n-- 6. 状态快照 (信息项) --")
    g, s, a = last.get("GPS_RAW_INT"), last.get("SYS_STATUS"), last.get("ATTITUDE")
    if g:
        fix = ["NO GPS", "NO FIX", "2D", "3D", "DGPS", "RTK Float", "RTK Fixed", "STATIC"]
        print(f"  GPS: {fix[g.fix_type] if g.fix_type < len(fix) else g.fix_type}"
              f" 星数={g.satellites_visible}")
    if s:
        print(f"  电池: {s.voltage_battery/1000:.2f}V 剩余={s.battery_remaining}%")
    if a:
        import math
        print(f"  姿态: roll={math.degrees(a.roll):+.1f}° pitch={math.degrees(a.pitch):+.1f}°"
              f" yaw={math.degrees(a.yaw):+.1f}°")

    master.close()
    print("\n== 结论 ==")
    hard_fail = [n for n, lv, ok, _ in results if lv == "HARD" and not ok]
    warns = [n for n, lv, ok, _ in results if lv == "WARN" and not ok]
    if hard_fail:
        print(f"FAIL: 链路不通/不达标 → {', '.join(hard_fail)}")
        sys.exit(1)
    print("PASS: 飞控链路正常 ✓" + (f"（告警: {', '.join(warns)}）" if warns else ""))
    sys.exit(0)


if __name__ == "__main__":
    main()
