#!/usr/bin/env python3
"""MAVLink 抓包探针：直连 5763 记录 ARM/DISARM/模式切换的来源（排障用）。"""
import time

from pymavlink import mavutil

m = mavutil.mavlink_connection('tcp:127.0.0.1:5763', source_system=251)
m.wait_heartbeat(timeout=15)
print('sniffer up', flush=True)
t0 = time.time()
while time.time() - t0 < 300:
    msg = m.recv_match(blocking=True, timeout=2)
    if msg is None:
        continue
    t = msg.get_type()
    if t in ('COMMAND_LONG', 'COMMAND_INT'):
        print('%.1f CMD src=%d,%d cmd=%d p1..7=%s'
              % (time.time() - t0, msg.get_srcSystem(), msg.get_srcComponent(),
                 msg.command, list(msg.__dict__.get(f'param{i}', 0) for i in range(1, 8))),
              flush=True)
    elif t == 'STATUSTEXT':
        print('%.1f TEXT src=%d sev=%d %s'
              % (time.time() - t0, msg.get_srcSystem(), msg.severity, msg.text), flush=True)
