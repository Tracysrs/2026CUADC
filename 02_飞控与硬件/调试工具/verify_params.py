# -*- coding: utf-8 -*-
"""飞控参数审计工具
用法:
  python verify_params.py --export            # 导出飞控全参数备份（时间戳文件名）
  python verify_params.py --diff 参数文件.param # 飞控实际值 vs 参数文件 逐项比对
  python verify_params.py --export --diff 设计/V6X_ardupilot_params.param  # 两者都做
  COM 号漂移时用 --port 指定（默认 COM5）: python verify_params.py --port COM17 --export
对应方案 §10.2-6「配置与代码一致性」检查；刷固件/导参数前先 --export 备份。
"""
import argparse
import sys
import time
from datetime import datetime

from pymavlink import mavutil

PORT = "COM5"
BAUD = 115200
PARAM_TIMEOUT = 15.0      # 等第一个参数的最长时间
QUIET_PERIOD = 2.0        # 流停顿多少秒后开始补拉缺失参数
MAX_RETRY_ROUNDS = 30     # 按索引补拉的最大轮数


def connect(port):
    master = mavutil.mavlink_connection(port, baud=BAUD, timeout=5)
    hb = master.wait_heartbeat(timeout=15)
    if hb is None:
        print("ERROR: 未收到心跳，检查接线和串口")
        sys.exit(1)
    print(f"已连接: system_id={master.target_system} component_id={master.target_component}")
    return master


def download_all_params(master):
    """全量下载参数；飞控忙时会暂停参数流导致缺口，按 param_count/param_index 补齐"""
    master.mav.param_request_list_send(master.target_system, master.target_component)
    params = {}        # name -> value
    by_index = {}      # param_index -> name
    total = None
    t0 = time.time()
    last_new = time.time()
    retries = 0
    while True:
        msg = master.recv_match(type="PARAM_VALUE", blocking=False)
        if msg:
            raw = msg.param_id
            key = raw.rstrip(b"\x00").decode("utf-8", "replace") if isinstance(raw, bytes) else raw.rstrip("\x00")
            if key not in params:
                last_new = time.time()
            params[key] = msg.param_value
            by_index[msg.param_index] = key
            total = msg.param_count
            if len(params) % 100 == 0:
                print(f"  已下载 {len(params)}/{total} 个参数...")

        if params and time.time() - last_new > QUIET_PERIOD and total:
            missing = [i for i in range(total) if i not in by_index]
            if not missing:
                break
            if retries >= MAX_RETRY_ROUNDS:
                print(f"WARNING: {len(missing)} 个参数重拉 {MAX_RETRY_ROUNDS} 轮仍缺失，放弃")
                break
            retries += 1
            last_new = time.time()
            for i in missing[:80]:  # 每轮补拉一小批
                master.mav.param_request_read_send(
                    master.target_system, master.target_component, b"\x00" * 16, i)

        if time.time() - t0 > PARAM_TIMEOUT and not params:
            print("ERROR: 超时未收到任何参数")
            sys.exit(1)
        if total and len(params) > total + 500:
            break
        time.sleep(0.02)
    print(f"  共下载 {len(params)} 个参数 (固件报告总数 {total})")
    return params


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


def values_match(a, b):
    if a == b:
        return True
    return abs(a - b) <= max(1e-6, abs(b) * 1e-4)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--export", action="store_true", help="导出飞控全参数备份")
    ap.add_argument("--diff", metavar="PARAM_FILE", help="与参数文件逐项比对")
    ap.add_argument("--port", default=PORT, help="COM 口（默认 COM5，号漂移时先扫端口）")
    args = ap.parse_args()
    if not (args.export or args.diff):
        ap.print_help()
        sys.exit(1)

    master = connect(args.port)
    print("正在下载飞控全参数...")
    fc = download_all_params(master)
    master.close()

    if args.export:
        out = f"param_backup_{datetime.now().strftime('%Y%m%d_%H%M%S')}.param"
        with open(out, "w", encoding="utf-8") as f:
            f.write(f"# 飞控全参数备份 {datetime.now().isoformat()} 共{len(fc)}项\n")
            for k in sorted(fc):
                f.write(f"{k},{fc[k]}\n")
        print(f"备份已保存: {out}")

    if args.diff:
        expected = parse_param_file(args.diff)
        diff, missing = [], []
        for k, want in expected.items():
            if k not in fc:
                missing.append(k)
            elif not values_match(fc[k], want):
                diff.append((k, fc[k], want))
        print(f"\n===== 比对结果: 参数文件 {args.diff} vs 飞控 =====")
        print(f"文件要求 {len(expected)} 项 | 一致 {len(expected) - len(diff) - len(missing)} | 不一致 {len(diff)} | 飞控上不存在 {len(missing)}")
        if diff:
            print(f"\n-- 值不一致（{len(diff)} 项）--")
            for k, now, want in sorted(diff):
                print(f"  {k}: 飞控={now} 文件={want}")
        if missing:
            print(f"\n-- 飞控上不存在（{len(missing)} 项，检查参数名/固件版本）--")
            for k in sorted(missing):
                print(f"  {k}={expected[k]}")
        if not diff and not missing:
            print("\n✅ 全部一致")


if __name__ == "__main__":
    main()
