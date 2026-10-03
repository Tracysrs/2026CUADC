# -*- coding: utf-8 -*-
"""X20P RTK 验收监控器（只读）：fix 状态、收敛计时、静置漂移、修正龄期、Fix 率。

04 册 §3.3 四组验收测试的定量记录件——①台架（Fixed 收敛 ≥3 次）/③机上单RTK 自洽/
转正四组（固定点静置、同点断电重启、开阔慢速移动、地面中断修正）的数据都从这里出。
只读监控，不发任何配置/控制指令；改参走 MP 或专用脚本，勿在本工具上搭车。

监控位（对应判据）：
  收敛用时 = watch 开始到该实例首次 RTK固定(6) 的墙钟秒（飞机/模块上电时刻现场自记）
  Fix 率   = 会话内 fix≥5（浮点+固定）样本占比（RTK 实例）
  静置漂移 = 该实例首次 Fixed 原点的北/东向偏移（①/静置判据：10 分钟漂移 cm 级）
  掉出 RTK = 曾 Fixed 后又跌回 <5 的次数（中断修正/慢速移动判据的关注点）
  修正龄期 = GPS_RTK.time_last_ms / GPS2_RTK.time_last_baseline_ms 分位数
            （MAVLink 无 rtk_age 字段，两条 RTK 报文的时间字段名不对称；
            p95 突涨=数传拥堵征兆）

用法: python rtk_watch.py [COM口] [--baud N] [--seconds N] [--tag 名字]
  COM 口缺省自动找「ArduPilot」描述口（FC USB 直连）；走 915 数传时点名
  COM 口并带 --baud（SERIAL2_BAUD=57 → 57600）。不指定波特率时依次试 115200/57600。
  --seconds 缺省 600（=静置判据时长）；Ctrl+C 提前结束照常出报告。
  --tag 给本次测试起名（进留档文件名，如 --tag restart2 = 断电重启第 2 次）。
留档: _logtmp/rtk_watch_<tag>_<时间戳>.jsonl（白名单外，本地留证不入库）
"""

import argparse
import json
import math
import os
import serial.tools.list_ports
import sys
import time

from pymavlink import mavutil

FIX_NAMES = ["无GPS", "无定位", "2D", "3D", "DGPS", "RTK浮点", "RTK固定", "静态", "PPP"]
# 报文名 -> 实例名（GPS_RAW_INT=实例0/GPS1，GPS2_RAW=实例1/GPS2；X20P 走 CAN 或串口
# 都可能占任一实例——判据看谁出 RTK 解，不预设位置）
INST_OF = {"GPS_RAW_INT": "GPS1", "GPS2_RAW": "GPS2"}
# 龄期字段两条报文不对称（2026-10-01 定案）：GPS_RTK.time_last_ms=距上次改正 ms，
# GPS2_RTK.time_last_baseline_ms=距上次基线解算 ms
AGE_FIELD_OF = {"GPS_RTK": "time_last_ms", "GPS2_RTK": "time_last_baseline_ms"}
PARAMS = ["GPS1_TYPE", "GPS2_TYPE", "GPS_AUTO_SWITCH",
          "CAN_P1_DRIVER", "CAN_D1_PROTOCOL", "GPS_AUTO_CONFIG"]


def new_state():
    return {"fix": None, "sats": None, "eph": None, "lat": None, "lon": None,
            "n": 0, "n_rtk": 0, "t_first": None, "t_float": None, "t_fix": None,
            "transitions": [], "drops": 0, "origin": None, "last_drift": None,
            "max_drift": 0.0}


def enu_m(lat, lon, lat0, lon0):
    mlat = 111132.92 - 559.82 * math.cos(2 * math.radians(lat0)) \
        + 1.175 * math.cos(4 * math.radians(lat0))
    mlon = 111412.84 * math.cos(math.radians(lat0)) \
        - 93.5 * math.cos(3 * math.radians(lat0))
    return (lat - lat0) * mlat, (lon - lon0) * mlon


def find_port():
    for p in serial.tools.list_ports.comports():
        if "ArduPilot" in (p.description or ""):
            return p.device
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default=None)
    ap.add_argument("--baud", type=int, default=None,
                    help="显式波特率（数传 57600 / FC USB 115200）；缺省依次试 115200/57600")
    ap.add_argument("--seconds", type=int, default=600)
    ap.add_argument("--tag", default="test")
    a = ap.parse_args()

    port = a.port or find_port()
    if not port:
        ports = ", ".join(f"{p.device}({p.description})" for p in
                          serial.tools.list_ports.comports()) or "无"
        sys.exit(f"没找到 ArduPilot 描述的串口。在位口：{ports}\n"
                 "飞控 USB 没插？走数传就点名 COM 口（如 python rtk_watch.py COM10 --baud 57600）"
                 "；或 Mission Planner 正占口（先断开 MP）")
    m = None
    for baud in ([a.baud] if a.baud else (115200, 57600)):
        try:
            cand = mavutil.mavlink_connection(port, baud=baud, timeout=5)
            hb = cand.wait_heartbeat(timeout=8)
        except Exception as e:
            print(f"{port}@{baud}: 打不开/异常 {e}")
            continue
        if hb:
            m = cand
            break
        cand.close()
        print(f"{port}@{baud}: 无心跳")
    if m is None:
        sys.exit(f"{port} 无 MAVLink 心跳——查：①是否飞控口/波特率 ②Mission Planner 是否占口"
                 "（先断开 MP 再跑本脚本）")
    print(f"已连接 {port}@{baud}（watch 开始 {time.strftime('%H:%M:%S')}，"
          f"时长 {a.seconds}s；飞机/模块上电时刻请现场自记）")

    params = {}
    for name in PARAMS:
        m.mav.param_request_read_send(m.target_system, m.target_component,
                                      name.encode(), -1)
    for mid, iv in [(24, 500000), (124, 500000), (127, 1000000), (128, 1000000)]:
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                0, mid, iv, 0, 0, 0, 0, 0)

    start = time.time()
    end = start + a.seconds
    state = {"GPS1": new_state(), "GPS2": new_state()}
    ages = {"GPS1": [], "GPS2": []}
    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_logtmp")
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, f"rtk_watch_{a.tag}_"
                            f"{time.strftime('%Y%m%d_%H%M%S')}.jsonl")
    logf = open(out_path, "a", encoding="utf-8")

    def rec(obj):
        logf.write(json.dumps(obj, ensure_ascii=False) + "\n")

    rec({"type": "start", "t": 0.0, "port": port, "tag": a.tag,
         "wall": time.strftime("%Y-%m-%d %H:%M:%S")})

    # 参数收包与 GPS 流并行收 6s
    endp = time.time() + 6
    while time.time() < endp:
        msg = m.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        t = msg.get_type()
        if t == "PARAM_VALUE":
            k = msg.param_id.rstrip(b"\x00").decode() \
                if isinstance(msg.param_id, bytes) else msg.param_id
            params.setdefault(k, msg.param_value)
        elif t in INST_OF:
            handle_gps(msg, t, state, start, rec)

    print("\n--- 机上 GPS 参数 ---")
    for k in PARAMS:
        v = params.get(k)
        note = ""
        if k == "GPS1_TYPE":
            note = {0: "无", 1: "串口AUTO(NEO3现役位)", 2: "串口UBX", 9: "DroneCAN"}.get(int(v), "")
        if k == "GPS2_TYPE":
            note = {0: "未激活", 2: "串口UBX", 9: "DroneCAN(X20P挂此处=双实例对照)"}.get(int(v), "")
        if k == "GPS_AUTO_SWITCH":
            note = {0: "首选GPS1", 1: "UseBest"}.get(int(v), "")
        print(f"  {k} = {v if v is not None else '读不到'} {note}")
    rec({"type": "params", "t": round(time.time() - start, 1), "params": params})

    # 主盯梢
    last_status = 0.0
    try:
        while time.time() < end:
            msg = m.recv_match(blocking=True, timeout=0.5)
            if msg is None:
                continue
            t = msg.get_type()
            if t in INST_OF:
                handle_gps(msg, t, state, start, rec)
            elif t in AGE_FIELD_OF:
                name = "GPS2" if t == "GPS2_RTK" else "GPS1"
                val = getattr(msg, AGE_FIELD_OF[t], None)
                if val is not None:
                    ages[name].append(val)
                    rec({"type": "rtk_age", "t": round(time.time() - start, 1),
                         "inst": name, "age_ms": val})
            now = time.time()
            if now - last_status >= 2.0:
                last_status = now
                print(status_line(state, ages, now - start), flush=True)
    except KeyboardInterrupt:
        print("\n（Ctrl+C 提前收摊）")

    m.close()
    logf.close()
    report(state, ages, a, params, out_path)


def handle_gps(msg, mtype, state, start, rec):
    name = INST_OF[mtype]
    st = state[name]
    ft = msg.fix_type
    now = time.time()
    rel = round(now - start, 1)
    if st["fix"] is None:
        st["t_first"] = rel
    if ft != st["fix"]:
        st["transitions"].append({"t": rel, "from": st["fix"], "to": ft})
        if st["fix"] is not None and st["fix"] >= 5 and ft < 5:
            st["drops"] += 1
            print(f"  !!! {name} 掉出 RTK（≥5→{ft}）@ {rel}s", flush=True)
        if ft == 5 and st["t_float"] is None:
            st["t_float"] = rel
        if ft == 6 and st["t_fix"] is None:
            st["t_fix"] = rel
            st["origin"] = (msg.lat / 1e7, msg.lon / 1e7)
            print(f"  *** {name} 首次 RTK固定 @ {rel}s（收敛用时，自 watch 起）",
                  flush=True)
        st["fix"] = ft
    st["n"] += 1
    if ft >= 5:
        st["n_rtk"] += 1
    st["sats"] = msg.satellites_visible
    st["eph"] = msg.eph
    st["lat"] = msg.lat / 1e7
    st["lon"] = msg.lon / 1e7
    if st["fix"] == 6 and st["origin"]:
        dn, de = enu_m(st["lat"], st["lon"], *st["origin"])
        st["last_drift"] = (dn, de)
        st["max_drift"] = max(st["max_drift"], math.hypot(dn, de))
    rec({"type": "gps", "t": rel, "inst": name, "fix": ft,
         "sats": st["sats"], "eph": st["eph"],
         "lat": st["lat"], "lon": st["lon"]})


def fix_str(st):
    if st["fix"] is None:
        return "无报文"
    ft = st["fix"]
    hdop = f"{st['eph'] / 100:.2f}" if st["eph"] not in (None, 65535, 0) else "—"
    return f"{FIX_NAMES[ft] if ft < len(FIX_NAMES) else ft} {st['sats']}星 HDOP{hdop}"


def status_line(state, ages, rel):
    parts = [f"[{int(rel):3d}s]"]
    for name in ("GPS1", "GPS2"):
        parts.append(f"{name}: {fix_str(state[name])}")
    have_age = [n for n in ("GPS1", "GPS2") if ages[n]]
    if have_age:
        parts.append("龄期 " + " ".join(
            f"{n} {ages[n][-1] / 1000:.1f}s" for n in have_age))
    rtk = next((n for n in ("GPS1", "GPS2") if state[n]["fix"] == 6), None)
    if rtk and state[rtk]["last_drift"]:
        dn, de = state[rtk]["last_drift"]
        parts.append(f"漂移 N{dn:+.2f} E{de:+.2f} 峰{state[rtk]['max_drift']:.2f}m")
    return " | ".join(parts)


def report(state, ages, a, params, out_path):
    print("\n=== 验收报告（tag=%s）===" % a.tag)
    for name in ("GPS1", "GPS2"):
        st = state[name]
        label = f"{name}" + ("(X20P 实例)" if st["fix"] is not None and st["fix"] >= 5
                             else "")
        print(f"\n-- {label}")
        if st["fix"] is None:
            print("   全程无报文（实例未激活或接线/参数未落）")
            continue
        tl = " → ".join(f"{tr['t']}s:{'无' if tr['from'] is None else FIX_NAMES[tr['from']] if tr['from'] < len(FIX_NAMES) else tr['from']}→{FIX_NAMES[tr['to']] if tr['to'] < len(FIX_NAMES) else tr['to']}"
                        for tr in st["transitions"][:12])
        print(f"   时间线: {tl or '（状态无变化）'}")
        if st["t_fix"] is not None:
            print(f"   收敛用时（自 watch 起）: {st['t_fix']}s"
                  f"（首次浮点 {st['t_float']}s）")
        rate = 100.0 * st["n_rtk"] / st["n"] if st["n"] else 0
        print(f"   Fix 率（fix≥5 占比）: {st['n_rtk']}/{st['n']} = {rate:.1f}%"
              f"；掉出 RTK {st['drops']} 次")
        ag = ages[name]
        if ag:
            s = sorted(ag)
            fld = "修正龄期(GPS_RTK.time_last_ms)" if name == "GPS1" \
                else "基线龄期(GPS2_RTK.time_last_baseline_ms)"
            print(f"   {fld}: p50 {s[len(s) // 2] / 1000:.1f}s / "
                  f"p95 {s[int(len(s) * 0.95)] / 1000:.1f}s / max {s[-1] / 1000:.1f}s"
                  f"（n={len(s)}）")
        else:
            print("   修正龄期: 无 GPSx_RTK 报文（基站链未通或固件未出该报文）")
        if st["origin"]:
            print(f"   静置漂移: 末偏 N{st['last_drift'][0]:+.2f} "
                  f"E{st['last_drift'][1]:+.2f}m；峰值 {st['max_drift']:.3f}m")
            print(f"   ★ 原点 lat={st['origin'][0]:.7f} lon={st['origin'][1]:.7f}"
                  "（断电重启测试：下次同点位重启的新原点与此对比，应差 <数 cm）")
    print(f"\n留档: {out_path}")
    print("判据对照（04 册 §3.3）：①台架=Fixed 收敛分钟级（≥3 次）；②静态对比=Fix 后"
          "与基准差 <数 cm 且 10min 无漂移；③机上=与单点解自洽+EKF 无告警；④带宽="
          "注入期 NAV30 无 >50ms 空洞（MP 侧统计，本脚本不覆盖）")


if __name__ == "__main__":
    main()
