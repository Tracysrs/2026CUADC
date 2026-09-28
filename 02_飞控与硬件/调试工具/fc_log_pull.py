#!/usr/bin/env python3
"""从飞控 SD 卡拉取 dataflash 日志（mavftp）。

用法：fc_log_pull.py list               # 列 /APM/LOGS
      fc_log_pull.py get <name> [outdir]  # 拉取指定日志
"""
import os
import sys
import time

from pymavlink import mavftp, mavutil

PORT = os.environ.get("FC_PORT", "/dev/cuadc-fc")
BAUD = os.environ.get("FC_BAUD", "115200")


def connect():
    conn = mavutil.mavlink_connection(PORT, baud=BAUD,
                                      source_system=255, source_component=190)
    hb = conn.wait_heartbeat(timeout=10)
    if hb is None:
        sys.exit("错误：10s 未收到飞控心跳")
    print(f"心跳 OK（sys {conn.target_system}/comp {conn.target_component}）")
    return mavftp.MAVFTP(conn, target_system=conn.target_system,
                         target_component=conn.target_component)


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "list"
    f = connect()
    if mode == "list":
        # SD 卡目录是 /APM/LOGS 全大写——小写 /APM/Logs 报 FileDoesNotExist(09-28 实测)
        ret = f.cmd_list(["/APM/LOGS"])
        if ret.error_code != 0:
            # 日志多时(146+ 项)pymavlink 内部 5s 超时会切断列表,不代表卡缺席:
            # /、/APM 能列出即卡在位;正式拉取改走 mavftp CLI --burst_read_size 239
            sys.exit(f"list 失败: {ret}（目录大时易 5s 超时;卡在位与否看 /APM 能否列出 LOGS/STRG_BAK/TERRAIN）")
        entries = f.list_result or []
        for e in sorted(entries, key=lambda x: x.name):
            kind = "DIR " if e.is_dir else f"{e.size_b/1024:8.0f}KB"
            print(f"{kind}  {e.name}")
        print(f"共 {len(entries)} 项")
    elif mode == "get":
        name = sys.argv[2]
        outdir = sys.argv[3] if len(sys.argv) > 3 else "/tmp"
        remote = name if name.startswith("/") else f"/APM/LOGS/{name}"
        t0 = time.time()
        ret = f.cmd_get([remote, os.path.join(outdir, os.path.basename(name))])
        dt = time.time() - t0
        if ret.error_code != 0:
            sys.exit(f"get 失败: {ret}")
        local = os.path.join(outdir, os.path.basename(name))
        size = os.path.getsize(local)
        print(f"完成 {local}: {size/1024:.0f}KB，用时 {dt:.0f}s"
              f"（{size/1024/max(dt,0.1):.0f}KB/s）")
    else:
        sys.exit(f"未知模式 {mode}")


if __name__ == "__main__":
    main()
