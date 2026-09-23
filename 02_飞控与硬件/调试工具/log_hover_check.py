# -*- coding: utf-8 -*-
"""dataflash 悬停/倾斜检查：离地窗口的姿态、四电机 PWM 平衡、振动、罗盘、GPS 一页报告。

用法: python log_hover_check.py <日志.BIN> [离地阈值米，默认0.3]
只读分析，不动飞控。结论供左倾/右倾归因（机械不平/推力不平衡/操纵补偿）参考。
"""

import math
import sys

try:
    from pymavlink import dfreader as _dfr
    DFReader_binary = _dfr.DFReader_binary
except ImportError:
    from pymavlink.DFReader import DFReader_binary

EV = {10: "ARMED", 11: "DISARMED", 15: "Takeoff", 16: "LAND?",
      17: "NotLanding", 18: "TakeoffComplete"}


def norm3(x, y, z):
    return math.sqrt(x * x + y * y + z * z)


def stats(v):
    n = len(v)
    if n == 0:
        return None
    mean = sum(v) / n
    var = sum((x - mean) ** 2 for x in v) / n
    return mean, math.sqrt(var), min(v), max(v), n


def main():
    path = sys.argv[1]
    alt_th = float(sys.argv[2]) if len(sys.argv) > 2 else 0.3
    mlog = DFReader_binary(path)

    att, ctun, rcou, rcin = [], [], [], []
    vibe, mags, gps, modes, ev, msgs, errs = [], [], [], [], [], [], []
    t0_log = None
    while True:
        m = mlog.recv_match()
        if m is None:
            break
        t = m._timestamp
        if t0_log is None:
            t0_log = t
        ty = m.get_type()
        rel = t - t0_log
        if ty == "ATT":
            att.append((rel, m.Roll, m.DesRoll, m.Pitch, m.DesPitch))
        elif ty == "CTUN":
            ctun.append((rel, m.Alt, m.ThO, m.ThI))
        elif ty == "RCOU":
            rcou.append((rel, m.C1, m.C2, m.C3, m.C4))
        elif ty == "RCIN":
            rcin.append((rel, m.C1, m.C2, m.C3, m.C4))
        elif ty == "VIBE":
            vibe.append((rel, m.IMU, m.VibeX, m.VibeY, m.VibeZ, m.Clip))
        elif ty == "MAG":
            mags.append((rel, f"MAG{m.I + 1}", norm3(m.MagX, m.MagY, m.MagZ),
                         m.Health))
        elif ty == "GPS":
            gps.append((rel, m.Status, m.NSats, m.HDop))
        elif ty == "MODE":
            modes.append((rel, m.Mode))
        elif ty == "EV":
            ev.append((rel, EV.get(m.Id, m.Id)))
        elif ty == "MSG":
            msgs.append((rel, m.Message))
        elif ty == "ERR":
            errs.append((rel, m.Subsys, m.ECode))

    print(f"=== {path} ===")
    print(f"日志总时长 {t0_log and max(att[-1][0] if att else 0, 0):.0f}s")
    print(f"事件: {ev if ev else '（无 EV，看 MSG）'}")
    print(f"模式: {modes}")
    for rel, s in msgs[:12]:
        print(f"  [{rel:6.1f}s] MSG {s}")
    if len(msgs) > 12:
        print(f"  …（MSG 共 {len(msgs)} 条，只列前 12）")
    if errs:
        print(f"ERR: {errs[:10]}")

    if not ctun:
        sys.exit("无 CTUN 数据")
    motor_on = [x[0] for x in rcou if max(x[1:5]) > 1050]
    if motor_on:
        t_in, t_out = motor_on[0], motor_on[-1]
        basis = "电机输出>1050µs"
    else:
        airborne_ct = [(rel, alt, tho, thi) for rel, alt, tho, thi in ctun if alt > alt_th]
        if not airborne_ct:
            sys.exit("全程电机未转（RCOU≤1050µs）且无 CTUN 离地样本")
        t_in, t_out = airborne_ct[0][0], airborne_ct[-1][0]
        basis = f"CTUN.Alt>{alt_th}m（电机未动，仅供气压参考）"
    alts = [a for rel, a, _, _ in ctun if t_in <= rel <= t_out]
    print(f"\n运转窗口: {t_in:.1f}s ~ {t_out:.1f}s（{t_out-t_in:.1f}s，判据 {basis}），"
          f"高度 {min(alts):.2f}~{max(alts):.2f}m")

    def in_win(seq):
        return [x for x in seq if t_in <= x[0] <= t_out]

    a_win = in_win(att)
    if a_win:
        rolls = [x[1] for x in a_win]
        desr = [x[2] for x in a_win]
        pitch = [x[3] for x in a_win]
        s = stats(rolls)
        print(f"ROLL  均值 {s[0]:+.2f}°  std {s[1]:.2f}°  范围 [{s[2]:+.1f}, {s[3]:+.1f}]"
              f"（正=右倾，负=左倾）")
        s = stats(desr)
        print(f"DesRoll 均值 {s[0]:+.2f}°  std {s[1]:.2f}°（期望滚转=飞控补偿需求）")
        s = stats(pitch)
        print(f"PITCH 均值 {s[0]:+.2f}°  std {s[1]:.2f}°")
    r_win = in_win(rcou)
    if r_win:
        print("\n四电机 PWM（离地窗口）：")
        means = []
        for i in range(4):
            s = stats([x[1 + i] for x in r_win])
            means.append(s[0])
            print(f"  C{i+1}: 均值 {s[0]:.0f}µs  std {s[1]:.0f}  "
                  f"范围 [{s[2]:.0f}, {s[3]:.0f}]")
        base = sum(means) / 4
        print(f"  四电机均值偏差: " +
              "  ".join(f"C{i+1} {means[i]-base:+.0f}µs" for i in range(4)) +
              "（|偏差|>20µs 提示该路推力/桨不匹配）")
    i_win = in_win(rcin)
    if i_win:
        s = stats([x[1] for x in i_win])
        print(f"\nRoll 杆量 均值 {s[0]:.0f}µs（>1520=持续向右补偿, <1480=持续向左）")
        s = stats([x[2] for x in i_win])
        print(f"Pitch 杆量 均值 {s[0]:.0f}µs")
    c_win = in_win(ctun)
    if c_win:
        s = stats([x[2] for x in c_win])
        s2 = stats([x[3] for x in c_win])
        print(f"油门输出 ThO 均值 {s[0]*100:.1f}%  油门输入 ThI 均值 {s2[0]*100:.1f}%")
    if vibe:
        v_win = [x for x in vibe if x[0] >= t_in - 1] or vibe
        for imu in sorted({x[1] for x in v_win}):
            rows = [x for x in v_win if x[1] == imu]
            vx = max(rows, key=lambda r: max(abs(r[2]), abs(r[3]), abs(r[4])))
            clip = max(r[5] for r in rows)
            print(f"振动 IMU{imu}: |Vibe| 峰 {max(abs(vx[2]),abs(vx[3]),abs(vx[4])):.2f} m/s/s"
                  f"（>30 异常） Clip 累计 {clip:.0f}（>0 即 IMU 削波）")
    for inst in ("MAG1", "MAG2", "MAG3"):
        mm = [x for x in mags if x[1] == inst]
        if mm:
            m_win = [x for x in mm if t_in <= x[0] <= t_out] or mm
            s = stats([x[2] for x in m_win])
            h = "健康" if all(x[3] == 1 for x in mm) else "有不健康样本!"
            print(f"罗盘 {inst}: 场模长均值 {s[0]:.0f} mG  std {s[1]:.0f}  {h}"
                  f"（两颗差 >30~50 查校准）")
    if gps:
        g_win = [x for x in gps if t_in <= x[0] <= t_out] or gps[-5:]
        st = g_win[-1]
        ns = max(x[2] for x in g_win)
        print(f"GPS（窗末）: Status={st[1]} 星 {st[2]}（峰 {ns}） HDOP {st[3]:.2f}")


if __name__ == "__main__":
    main()
