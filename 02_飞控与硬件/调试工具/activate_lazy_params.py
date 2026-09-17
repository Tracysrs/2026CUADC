# -*- coding: utf-8 -*-
"""激活懒加载参数组并重启飞控，发现新固件的真实参数名
写入 RNGFND1_TYPE=20、PLND_ENABLED=1（均为 V6X_ardupilot_params.param 中本来就要求的值），
重启后重新下载全参数，打印 RNGFND1_*/PLND_*/GUID* 的完整名单。
"""
import sys
import time

from pymavlink import mavutil

import verify_params as vp

PORT = "COM5"
BAUD = 115200

MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246


def set_param_wait(master, name, value):
    master.param_set_send(name, float(value))
    t0 = time.time()
    while time.time() - t0 < 3:
        msg = master.recv_match(type="PARAM_VALUE", blocking=False)
        if msg:
            key = msg.param_id.rstrip(b"\x00").decode() if isinstance(msg.param_id, bytes) else msg.param_id
            if key == name:
                print(f"  {name} -> {msg.param_value} ✓")
                return True
        time.sleep(0.02)
    print(f"  {name} 写入无确认 ✗")
    return False


def main():
    master = vp.connect()

    print("写入激活参数:")
    ok1 = set_param_wait(master, "RNGFND1_TYPE", 20)
    ok2 = set_param_wait(master, "PLND_ENABLED", 1)
    if not (ok1 and ok2):
        print("参数写入失败，不重启")
        sys.exit(1)

    print("重启飞控...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0)
    master.close()
    time.sleep(15)

    print("重新连接并下载全参数...")
    master = vp.connect()
    params = vp.download_all_params(master)
    master.close()

    print("\n=== 激活后的参数名 ===")
    for pat in ("RNGFND1", "PLND", "GUID"):
        hits = sorted((k, v) for k, v in params.items() if k.startswith(pat))
        print(f"--- {pat}* ({len(hits)} 项) ---")
        for k, v in hits:
            print(f"  {k} = {v}")


if __name__ == "__main__":
    main()
