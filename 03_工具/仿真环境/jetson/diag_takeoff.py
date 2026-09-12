#!/usr/bin/env python3
"""Diagnose takeoff rejection: check mode, send takeoff, correlate ACK by command id,
print STATUSTEXT reasons."""
import time

from pymavlink import mavutil

m = mavutil.mavlink_connection("udpin:0.0.0.0:14550", timeout=5)
m.wait_heartbeat(timeout=15)
print(f"heartbeat: sys={m.target_system} comp={m.target_component}")

# flush backlog
t0 = time.time()
while time.time() - t0 < 1.0:
    m.recv_match(blocking=False)

# current mode from heartbeat
hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
if hb:
    print(f"mode custom_mode={hb.custom_mode} (GUIDED=4, base={hb.base_mode:#x})")

# arm state
# set GUIDED + arm
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 0, 0, 0, 0, 0)
ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
print(f"set GUIDED ack: {ack.result if ack else None}")
time.sleep(1)
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
print(f"arm ack: {ack.result if ack else None}")
time.sleep(1)

msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
if msg:
    print(f"amsl={msg.alt/1000.0:.2f} m rel={msg.relative_alt/1000.0:.2f} m")
    target = msg.alt / 1000.0 + 8.0
else:
    target = 8.0
print(f"takeoff target amsl = {target:.2f}")

m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                        0, 0, 0, 0, 0, 0, target)

t0 = time.time()
while time.time() - t0 < 10:
    msg = m.recv_match(blocking=True, timeout=2)
    if msg is None:
        continue
    t = msg.get_type()
    if t == "COMMAND_ACK":
        print(f"ACK cmd={msg.command} result={msg.result}")
    elif t == "STATUSTEXT":
        print(f"STATUSTEXT: {msg.severity} {msg.text}")
    elif t == "GLOBAL_POSITION_INT":
        print(f"rel_alt={msg.relative_alt/1000.0:.2f}")
    if time.time() - t0 > 6 and msg.get_type() == "GLOBAL_POSITION_INT" and msg.relative_alt > 3000:
        print("CLIMBING!")
        break

# land + disarm for safety
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
time.sleep(2)
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
print("land+disarm sent")
