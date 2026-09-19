# -*- coding: utf-8 -*-
"""参数基线导入工具：脚本化替代 Mission Planner 手动「载入参数→写入→重启→再载入→再重启」

用法（在 02_飞控与硬件/调试工具/ 目录下）：
  python load_params.py COM17 ../V6X_ardupilot_params.param              # 默认两遍，每遍后重启
  python load_params.py COM17 ../RC标定_iA6B.param --passes 1            # 单遍（无懒加载组时用）
  python load_params.py COM17 ../V6X_ardupilot_params.param --no-reboot  # 导完不重启（连调时慎用）

为什么必须两遍：4.7-beta 懒加载参数组（PLND_TYPE、RNGFND1_* 子参数等）要等
父开关写入并重启后才出现在参数表，第一遍写入时会被静默忽略——详见
06_使用说明书/03_飞控配置与校准.md §2.2。导完用 verify_params.py --diff 验收。
"""
import argparse
import sys
import time

from pymavlink import mavutil

BAUD = 115200
WRITE_TIMEOUT = 3.0      # 单参数等待回读
WRITE_RETRIES = 3        # 单参数重试次数
RECONNECT_WAIT = 60.0    # 重启后重连总时限


def connect(port, timeout=15.0):
    """连飞控等心跳；重启后端口恢复慢，带重试窗口"""
    deadline = time.time() + timeout
    while True:
        try:
            master = mavutil.mavlink_connection(port, baud=BAUD, timeout=5)
            hb = master.wait_heartbeat(timeout=8)
            if hb is not None:
                print(f"已连接 {port}: system_id={master.target_system} "
                      f"component_id={master.target_component}")
                return master
            master.close()
        except Exception as e:
            print(f"  连接 {port} 失败: {e}")
        if time.time() > deadline:
            return None
        print("  3 秒后重试……")
        time.sleep(3)


def decode_id(raw):
    return raw.rstrip(b"\x00").decode("utf-8", "replace") if isinstance(raw, bytes) \
        else raw.rstrip("\x00")


def write_param(master, name, value):
    """写入单参数并等 PARAM_VALUE 回读确认；值不符或超时返回 False"""
    for _ in range(WRITE_RETRIES):
        master.mav.param_set_send(
            master.target_system, master.target_component,
            name.encode("ascii")[:16].ljust(16, b"\x00"), value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        t0 = time.time()
        while time.time() - t0 < WRITE_TIMEOUT:
            msg = master.recv_match(type="PARAM_VALUE", blocking=False)
            if msg and decode_id(msg.param_id) == name:
                if abs(msg.param_value - value) <= max(1e-6, abs(value) * 1e-4):
                    return True
                print(f"  {name}: 回读 {msg.param_value} ≠ 写入 {value}，重试")
                break
            time.sleep(0.02)
    return False


def parse_param_file(path):
    expected = {}
    with open(path, encoding="utf-8") as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p.strip() for p in line.split(",")]
            if len(parts) < 2:
                print(f"  警告: {path}:{lineno} 无法解析 -> {line}")
                continue
            expected[parts[0]] = float(parts[1])
    return expected


def load_pass(master, expected):
    ok, missing = 0, []
    total = len(expected)
    for i, (name, value) in enumerate(expected.items(), 1):
        if write_param(master, name, value):
            ok += 1
        else:
            missing.append(name)
        if i % 20 == 0:
            print(f"  进度 {i}/{total}")
    return ok, missing


def reboot(master, port):
    print("\n重启飞控……")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1,
        0, 0, 0, 0, 0, 0)
    master.close()
    time.sleep(5)
    nxt = connect(port, timeout=RECONNECT_WAIT)
    if nxt is None:
        sys.exit("ERROR: 重启后连不上（软重启卡 bootloader 是已知现象）——"
                 "拔插 USB 等枚举恢复后，重新运行本命令继续下一遍")
    return nxt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", help="COM 口（号漂移先扫端口）")
    ap.add_argument("param_file", help="参数文件（# 开头为注释）")
    ap.add_argument("--passes", type=int, default=2, help="导入遍数（默认 2）")
    ap.add_argument("--no-reboot", action="store_true", help="导完不重启")
    args = ap.parse_args()

    expected = parse_param_file(args.param_file)
    print(f"参数文件 {args.param_file} 共 {len(expected)} 项，计划 {args.passes} 遍")

    master = connect(args.port)
    if master is None:
        sys.exit("ERROR: 未收到心跳，检查接线/串口/Mission Planner 是否占用")
    for p in range(1, args.passes + 1):
        print(f"\n===== 第 {p}/{args.passes} 遍导入 =====")
        ok, missing = load_pass(master, expected)
        print(f"本遍: 写入确认 {ok}/{len(expected)}"
              + (f"，未确认 {len(missing)} 项" if missing else ""))
        if missing:
            print("  未确认清单: " + ", ".join(missing))
        if p < args.passes or not args.no_reboot:
            master = reboot(master, args.port)

    if missing:
        print("\n⚠️ 仍有未确认项：懒加载组先跑满两遍；再不齐用 "
              "verify_params.py --diff 对账（参数名/固件版本问题会单独列出）")
    else:
        print("\n✅ 全部写入并回读确认；验收请跑: "
              f"python verify_params.py --port {args.port} --diff {args.param_file}")
    master.close()


if __name__ == "__main__":
    main()
