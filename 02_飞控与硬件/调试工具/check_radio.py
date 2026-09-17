#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""check_radio.py — 地面数传电台诊断（只读不改参）

用法:
    python check_radio.py [COM口] [波特率]      # 默认 COM10 57600

做什么:
    1) 链路活性    : 3 秒内 COM 口是否收到 MAVLink 帧（有心跳 = 飞控→机载台→433→地面台→PC 下行通）
    2) AT 命令模式 : 发 "+++\\r"（远航 X6 实测 X-Rock 固件需要回车，裸 +++ 无效）
    3) 电台身份    : ATI/ATI2/ATI3/ATI4，最后 ATO 退出并验证

⚠️ 已知坑（2026-09-12 实测，远航 3DR Radio X6 / X-Rock 4.0 固件）:
    * **不要给 X-Rock 4.0 发 ATI5** —— 固件不响应该转储且会把命令解析器挂死，
      挂死后电台对一切串口输入（含 +++）全哑，只能重插 USB 恢复。本脚本已剔除 ATI5。
    * AT 进入要求 +++ 前后各 ~1-2 秒串口静默；链路上心跳每 ~0.97s 一帧，
      后静默窗口必被打断 → **链路活着时基本进不去**（实测 25 次成 1 次）。
      要读/配电台参数：先给机载端断电，只留地面 USB 棒插电脑。
    * Mission Planner 1.3.83 的 SiK 电台页对本电台**不可用**：即使口对、波特率对、
      链路静默，其 RFD900.TSession 代码也必然抛"端口被关闭"异常。配置电台用
      本脚本核身 + 3DR Radio Config 独立工具，别用 MP 电台页。

依赖: pip install pyserial
"""
import re
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM10"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 57600


def readable(b):
    t = "".join(chr(x) if 32 <= x < 127 or x in (13, 10) else "." for x in b)
    return re.sub(r"\.{4,}", " ~ ", t).strip()


def main():
    try:
        g = serial.Serial(PORT, BAUD, timeout=0.3)
    except Exception as e:
        print("[X] 打不开 %s: %s" % (PORT, e))
        print("    占用中 = 别的程序(Mission Planner?)开着; 没设备 = 驱动/线缆")
        return 1
    print("[1] %s @ %d 打开成功" % (PORT, BAUD))

    # --- 链路活性 ---
    g.reset_input_buffer()
    time.sleep(3.0)
    data = g.read(8192)
    n_frames = data.count(b"\xfd")
    print("[2] 链路下行: 3 秒收到 %d 字节 / %d 个 MAVLink 帧 -> %s"
          % (len(data), n_frames, "通(心跳在流)" if n_frames else "静默(机载端没上电/没插好)"))

    # --- AT 进入: 长静默 + 重试 ---
    print("[3] 尝试进入 AT 命令模式（链路活跃时成功率极低, 失败请先给机载端断电再跑一次）")
    entered = False
    for i in range(5):
        time.sleep(2.5)
        g.reset_input_buffer()
        g.write(b"+++\r")  # X-Rock 需要回车; 裸 +++ 无应答
        time.sleep(1.5)
        r = g.read(4096)
        if b"OK" in r:
            entered = True
            print("    +++\\r 第 %d 次尝试 -> OK, 已进入" % (i + 1))
            break
        print("    +++\\r 第 %d 次尝试无应答" % (i + 1))

    if not entered:
        print("[4] 结论: 电台不应答 AT。两种可能:")
        print("    a) 链路心跳流打断了静默窗口 —— 给机载端断电后重跑;")
        print("    b) 电台命令解析器被挂死（此前发过不支持的 AT 指令）—— 重插地面 USB 棒恢复。")
        g.close()
        return 2

    # --- 身份转储 (无 ATI5! 会挂死 X-Rock 固件) ---
    time.sleep(0.5)
    g.reset_input_buffer()
    ok = True
    for c in ["ATI", "ATI2", "ATI3", "ATI4"]:
        g.write((c + "\r").encode())
        time.sleep(1.0)
        r = g.read(4096)
        txt = readable(r)
        if not txt:
            ok = False
        print("    %-5s -> %s" % (c, txt if txt else "(空)"))

    # --- ATO 退出并验证 (X-Rock 的 ATO 静默退出不回 OK, 用 ATI 探测判定) ---
    g.reset_input_buffer()
    g.write(b"ATO\r")
    time.sleep(1.5)
    g.read(4096)
    g.reset_input_buffer()
    g.write(b"ATI\r")
    time.sleep(1.0)
    r = g.read(4096)
    if not r:
        print("[5] ATO 退出: 成功（ATI 无应答 = 已回透传模式；X-Rock 退出不回 OK 属正常）")
    else:
        g.write(b"ATO\r")
        time.sleep(1.5)
        g.read(4096)
        print("[5] ATO 退出: 首次未生效，已补发一次；若下次探测再遇全哑请重插 USB 棒")
    g.close()

    print("[6] DONE. 电台身份应以 ATI=X-Rock 4.0 为准%s"
          % ("" if ok else "（部分指令无应答，留意挂起风险）"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
