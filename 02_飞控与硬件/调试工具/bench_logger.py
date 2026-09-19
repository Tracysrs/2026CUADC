# 台架联测电压记录仪：10Hz 记录电压/油门/急停/解锁态，跑 180s 后自动退出
import time
from pymavlink import mavutil

m = mavutil.mavlink_connection('/dev/cuadc-fc', baud=115200, timeout=5)
hb = m.wait_heartbeat(timeout=10)
armed = bool(hb.base_mode & 128)
m.mav.command_long_send(m.target_system, m.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
    mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 100000, 0, 0, 0, 0, 0)
m.mav.command_long_send(m.target_system, m.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
    mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, 100000, 0, 0, 0, 0, 0)

out = open('/tmp/cuadc_bench.csv', 'w')
out.write('t,voltage,ch3,ch6,armed\n')
row = {'v': None, 'ch3': None, 'ch6': None}
end = time.time() + 180
last_write = 0
while time.time() < end:
    msg = m.recv_match(type=['SYS_STATUS', 'RC_CHANNELS', 'HEARTBEAT'], blocking=True, timeout=1)
    if not msg:
        continue
    t = msg.get_type()
    if t == 'SYS_STATUS':
        row['v'] = msg.voltage_battery / 1000.0
    elif t == 'RC_CHANNELS':
        row['ch3'] = msg.chan3_raw
        row['ch6'] = msg.chan6_raw
    elif t == 'HEARTBEAT':
        armed = bool(msg.base_mode & 128)
    if time.time() - last_write >= 0.1 and row['v'] is not None:
        out.write(f'{time.time():.1f},{row["v"]:.2f},{row["ch3"]},{row["ch6"]},{int(armed)}\n')
        out.flush()
        last_write = time.time()
out.close()
