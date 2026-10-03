# -*- coding: utf-8 -*-
"""参数确认导入（写一验一版）：大批量参数导入的可靠路径

用法（在 02_飞控与硬件/调试工具/ 目录下）：
  python load_params_fast.py COM17 ../参数备份/xxx.param        # 默认两遍，每遍后重启
  python load_params_fast.py COM17 xxx.param --passes 1         # 单遍（无懒加载组时用）

两个已知坑叠加出的设计（09-28 与 10-02 两日实证）：
1. 4.7-beta（compid=0 心跳）param_set 后的 PARAM_VALUE 回执经常缺失（11 册 §4
   「回执假阴性」），且 Mission Planner 按标准 compid 发的写参请求会被飞控忽略
   ——表现为「MP 写不了参数」；pymavlink 按心跳实际 compid 发则通。
2. 链路满负荷流送遥测时，无回执的盲写 PARAM_SET 会整批丢帧（30ms 间隔洪灌
   1122 项仅 577 项落上，10-02 实证）——盲写只适合链路空闲的小批量。
因此本脚本：每项 写入→主动 param_request_read 点名读回→不符/超时重试≤3 次
（读回即流控），每遍后重启（懒加载组两遍规则），终判交 verify_params.py --diff。
"""
import argparse
import sys
import time

from pymavlink import mavutil

BAUD = 115200
SETTLE = 0.03            # 写入后稍候再点名
READ_TIMEOUT = 0.6       # 单次点名读回等待
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
                      f"component_id={master.target_component}", flush=True)
                return master
            master.close()
        except Exception as e:
            print(f"  连接 {port} 失败: {e}", flush=True)
        if time.time() > deadline:
            return None
        print("  3 秒后重试……", flush=True)
        time.sleep(3)


def pid(raw):
    return raw.rstrip(b"\x00").decode("utf-8", "replace") if isinstance(raw, bytes) \
        else raw.rstrip("\x00")


def read_param(master, name, timeout=READ_TIMEOUT):
    """点名读单参数；超时返回 None"""
    master.mav.param_request_read_send(
        master.target_system, master.target_component,
        name.encode("ascii")[:16].ljust(16, b"\x00"), -1)
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = master.recv_match(type="PARAM_VALUE", blocking=False)
        if msg and pid(msg.param_id) == name:
            return msg.param_value
        time.sleep(0.02)
    return None


def write_param(master, name, value):
    """写入单参数并点名读回确认；不符或超时重试。返回 True=确认落上"""
    for _ in range(WRITE_RETRIES):
        master.mav.param_set_send(
            master.target_system, master.target_component,
            name.encode("ascii")[:16].ljust(16, b"\x00"), value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        time.sleep(SETTLE)
        got = read_param(master, name)
        if got is not None and abs(got - value) <= max(1e-6, abs(value) * 1e-4):
            return True
    return False


def load_pass(master, expected):
    ok, missing = 0, []
    total = len(expected)
    t0 = time.time()
    for i, (name, value) in enumerate(expected.items(), 1):
        if write_param(master, name, value):
            ok += 1
        else:
            missing.append(name)
        if i % 50 == 0:
            print(f"  进度 {i}/{total}（确认 {ok}）({time.time()-t0:.0f}s)", flush=True)
    print(f"本遍: 写入确认 {ok}/{total} ({time.time()-t0:.0f}s)"
          + (f"，未落 {len(missing)} 项" if missing else ""), flush=True)
    if missing:
        print("  未落清单: " + ", ".join(missing), flush=True)
    return missing


def reboot(master, port):
    print("\n重启飞控……", flush=True)
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


def parse_param_file(path):
    expected = {}
    with open(path, encoding="utf-8") as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p.strip() for p in line.split(",")]
            if len(parts) < 2:
                print(f"  警告: {path}:{lineno} 无法解析 -> {line}", flush=True)
                continue
            expected[parts[0]] = float(parts[1])
    return expected


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", help="COM 口（号漂移先扫端口）")
    ap.add_argument("param_file", help="参数文件（# 开头为注释）")
    ap.add_argument("--passes", type=int, default=2, help="导入遍数（默认 2）")
    ap.add_argument("--no-reboot", action="store_true", help="导完不重启")
    args = ap.parse_args()

    expected = parse_param_file(args.param_file)
    print(f"参数文件 {args.param_file} 共 {len(expected)} 项，计划 {args.passes} 遍（写一验一）",
          flush=True)

    master = connect(args.port)
    if master is None:
        sys.exit("ERROR: 未收到心跳，检查接线/串口/Mission Planner 是否占用")
    missing = []
    for p in range(1, args.passes + 1):
        print(f"\n===== 第 {p}/{args.passes} 遍导入 =====", flush=True)
        missing = load_pass(master, expected)
        if p < args.passes or not args.no_reboot:
            master = reboot(master, args.port)

    if missing:
        print("\n⚠️ 仍有未落项（懒加载组先跑满两遍；再不齐用 "
              "verify_params.py --diff 对账）", flush=True)
    else:
        print("\n✅ 全部写入并点名读回确认；验收（终判）: "
              f"python verify_params.py --port {args.port} --diff {args.param_file}",
              flush=True)
    master.close()


if __name__ == "__main__":
    main()
