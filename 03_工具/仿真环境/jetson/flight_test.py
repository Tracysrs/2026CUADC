#!/usr/bin/env python3
"""Flight smoke test over SITL+Gazebo: GUIDED -> arm -> takeoff 8m -> hover -> land.
Connects to sim_vehicle's MAVLink output on udp:127.0.0.1:14550.
"""
import sys
import time

from pymavlink import mavutil

TAKEOFF_HEIGHT_M = 8.0
POLL_SECONDS = 30


def wait_msg(m, name, timeout=10):
    msg = m.recv_match(type=name, blocking=True, timeout=timeout)
    if msg is None:
        print(f"  !! timeout waiting for {name}")
    return msg


def send_command(m, cmd, params, timeout=8):
    m.mav.command_long_send(m.target_system, m.target_component, cmd, 0, *params)
    ack = wait_msg(m, "COMMAND_ACK", timeout)
    if ack is None:
        return None
    return ack.result


def main():
    m = mavutil.mavlink_connection("udpin:0.0.0.0:14550", timeout=5)
    print("[1] waiting heartbeat (SITL)...")
    if m.wait_heartbeat(timeout=30) is None:
        print("FAIL: no heartbeat from SITL")
        return 1
    print(f"    OK system={m.target_system}, autopilot={m.target_component}")

    print("[2] waiting EKF/position (GPS fix + attitude)...")
    deadline = time.time() + 60
    got_gps = False
    while time.time() < deadline:
        msg = m.recv_match(type=["GPS_RAW_INT", "GLOBAL_POSITION_INT"],
                           blocking=True, timeout=5)
        if msg is None:
            continue
        t = msg.get_type()
        if t == "GPS_RAW_INT" and msg.fix_type >= 3:
            got_gps = True
            print(f"    GPS 3D fix, sats={msg.satellites_visible}")
            break
    if not got_gps:
        print("FAIL: no GPS fix in 60s")
        return 1
    time.sleep(3)  # let EKF settle

    print("[3] set GUIDED...")
    res = send_command(m, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                       (mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4,
                        0, 0, 0, 0, 0))  # custom_mode 4 = GUIDED (ArduCopter)
    print(f"    ACK result={res}")
    if res != 0:
        print("FAIL: set GUIDED")
        return 1

    print("[4] arm...")
    res = send_command(m, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, (1, 0, 0, 0, 0, 0, 0))
    print(f"    ACK result={res}")
    if res != 0:
        print("FAIL: arm (result=%s)" % res)
        return 1

    print("[5] takeoff to %.0f m..." % TAKEOFF_HEIGHT_M)
    msg = wait_msg(m, "GLOBAL_POSITION_INT", 5)
    cur_amsl = msg.alt / 1000.0 if msg else 0.0
    res = send_command(m, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                       (0, 0, 0, 0, 0, 0, cur_amsl + TAKEOFF_HEIGHT_M))
    print(f"    ACK result={res}")
    if res != 0:
        print("FAIL: takeoff cmd")
        return 1

    print(f"[6] climbing, polling {POLL_SECONDS}s...")
    max_rel = 0.0
    t0 = time.time()
    while time.time() - t0 < POLL_SECONDS:
        msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
        if msg is None:
            continue
        rel = msg.relative_alt / 1000.0
        max_rel = max(max_rel, rel)
        print(f"    t={time.time()-t0:4.1f}s rel_alt={rel:6.2f} m")
    print(f"    max rel_alt = {max_rel:.2f} m")
    flew = max_rel > 5.0

    print("[7] LAND...")
    send_command(m, mavutil.mavlink.MAV_CMD_NAV_LAND, (0, 0, 0, 0, 0, 0, 0))
    time.sleep(10)
    msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=3)
    if msg:
        print(f"    rel_alt after land ≈ {msg.relative_alt/1000.0:.2f} m")

    print("RESULT:", "FLIGHT_OK (climbed >5 m)" if flew else "FLIGHT_FAIL")
    return 0 if flew else 1


if __name__ == "__main__":
    sys.exit(main())
