#!/usr/bin/env python3
"""One-shot SITL flight: fix frame params if needed, GUIDED, arm, takeoff 8m, land.
Usage: python3 fly.py [connection]
  default: udpin:0.0.0.0:14550  (MAVProxy --out 桥接在跑时)
  bare SITL (reset_sim.sh 栈,无 MAVProxy): python3 fly.py tcp:127.0.0.1:5760
"""
import sys
import time

from pymavlink import mavutil


def wait_ack(m, cmd, timeout=6):
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=1)
        if msg and msg.command == cmd:
            return msg.result
    return None


def get_param(m, name, timeout=6):
    m.mav.param_request_read_send(m.target_system, m.target_component,
                                  name.encode("ascii"), -1)
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg and msg.param_id.strip("\x00 ") == name:
            return msg.param_value
    return None


def set_param(m, name, value):
    m.mav.param_set_send(m.target_system, m.target_component,
                         name.encode("ascii"), float(value),
                         mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.5)
    return get_param(m, name)


def main():
    conn = sys.argv[1] if len(sys.argv) > 1 else "udpin:0.0.0.0:14550"
    m = mavutil.mavlink_connection(conn, timeout=5)
    print(f"[0] link {conn}", flush=True)
    print("[1] heartbeat...", flush=True)
    if m.wait_heartbeat(timeout=30) is None:
        print("FAIL: no heartbeat")
        return 1
    print("    OK")

    # bare SITL streams heartbeat only; nobody requests streams (MAVProxy used to).
    print("[1.5] request data streams @10Hz...", flush=True)
    m.mav.request_data_stream_send(m.target_system, m.target_component,
                                   mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
    time.sleep(1)

    print("[2] frame params...")
    fc = get_param(m, "FRAME_CLASS")
    ft = get_param(m, "FRAME_TYPE")
    print(f"    FRAME_CLASS={fc} FRAME_TYPE={ft}")
    if fc is None or ft is None:
        print("FAIL: params unreadable (SITL still booting?)")
        return 1
    # Quad-X: FRAME_CLASS=1 (Quad), FRAME_TYPE=1 (X). gazebo-iris.parm sets 1/1.
    if int(fc) != 1 or int(ft) != 1:
        print("    fixing to QUAD(1)/X(1)...")
        set_param(m, "FRAME_CLASS", 1)
        set_param(m, "FRAME_TYPE", 1)
        print(f"    now FC={get_param(m, 'FRAME_CLASS')} FT={get_param(m, 'FRAME_TYPE')}")
        print("    NOTE: frame change needs reboot to rebuild motor matrix")

    print("[3] wait GPS 3D fix + EKF...")
    deadline = time.time() + 60
    while time.time() < deadline:
        msg = m.recv_match(type="GPS_RAW_INT", blocking=True, timeout=5)
        if msg and msg.fix_type >= 3:
            print(f"    GPS fix, sats={msg.satellites_visible}")
            break
    else:
        print("FAIL: no GPS fix")
        return 1
    time.sleep(4)

    print("[4] GUIDED...")
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4,
                            0, 0, 0, 0, 0)
    r = wait_ack(m, mavutil.mavlink.MAV_CMD_DO_SET_MODE)
    print(f"    ack={r}")
    if r != 0:
        print("FAIL: GUIDED")
        return 1

    print("[5] arm...")
    for attempt in range(3):
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                1, 0, 0, 0, 0, 0, 0)
        r = wait_ack(m, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
        print(f"    ack={r}")
        if r == 0:
            break
        time.sleep(2)
    else:
        print("FAIL: arm")
        return 1

    print("[6] takeoff 8 m...")
    # Copter treats takeoff param7 as ABOVE-HOME: pass 8.0 directly.
    # (旧写法 AMSL+8 在 SITL 家点海拔 584m 时目标变 ~592m,即 55m 爬升 bug)
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                            0, 0, 0, 0, 0, 0, 8.0)
    r = wait_ack(m, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)
    print(f"    ack={r}")

    print("[7] climb poll 30 s...")
    max_rel = 0.0
    t0 = time.time()
    while time.time() - t0 < 30:
        msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
        if msg is None:
            continue
        rel = msg.relative_alt / 1000.0
        max_rel = max(max_rel, rel)
        print(f"    t={time.time()-t0:4.1f}s rel={rel:7.2f} m", flush=True)
    print(f"    max rel = {max_rel:.2f} m")

    print("[8] LAND...")
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
    wait_ack(m, mavutil.mavlink.MAV_CMD_NAV_LAND)
    landed = False
    # 时间窗轮询(90s):recv_match 一次只吃一条消息,不能按迭代数限时长
    t0 = time.time()
    while time.time() - t0 < 90:
        msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
        if msg and msg.relative_alt / 1000.0 < 0.3:
            landed = True
            break
    print(f"    landed={landed}")
    if landed:  # 只在确认落地后上锁,空中绝不发 disarm
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                0, 0, 0, 0, 0, 0, 0)
        print("    disarm sent")

    ok = max_rel > 5.0 and max_rel < 20.0 and landed
    print("RESULT:", "FLIGHT_OK" if ok else "FLIGHT_FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
