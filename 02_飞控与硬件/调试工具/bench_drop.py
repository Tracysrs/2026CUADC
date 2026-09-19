# 台架联测投放执行（单进程独占串口）：等待解锁→怠速稳定 2s→自动投放，全程记电压
# 用法: python3 bench_drop.py <servo_ch> <release_pwm> <stow_pwm> <hold_s> <tag>
import sys, time
from pymavlink import mavutil

ch, rel, stow, hold, tag = int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]), float(sys.argv[4]), sys.argv[5]

m = mavutil.mavlink_connection('/dev/cuadc-fc', baud=115200, timeout=5)
hb = m.wait_heartbeat(timeout=10)
armed = bool(hb.base_mode & 128)
m.mav.command_long_send(m.target_system, m.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
    mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 50000, 0, 0, 0, 0, 0)

out = open('/tmp/cuadc_bench.csv', 'a')
def log(evt):
    out.write(f'{time.time():.1f},evt,{evt}\n'); out.flush()
    print(time.strftime('%H:%M:%S'), evt, flush=True)

if not armed:
    log(f'{tag}-等待解锁（最长 180s）')
    end = time.time() + 180
    while time.time() < end and not armed:
        msg = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg:
            armed = bool(msg.base_mode & 128)
    if not armed:
        log(f'{tag}-超时未解锁，退出')
        sys.exit(2)
log(f'{tag}-已解锁，怠速稳定 2s')
time.sleep(2)

def fire(pwm):
    m.mav.command_long_send(m.target_system, m.target_component, 183, 0, ch, pwm, 0, 0, 0, 0, 0)

log(f'{tag}-release-{rel}')
fire(rel)
t_end = time.time() + hold
while time.time() < t_end:
    msg = m.recv_match(type='SYS_STATUS', blocking=True, timeout=0.2)
    if msg:
        out.write(f'{time.time():.1f},{msg.voltage_battery/1000.0:.2f},release\n'); out.flush()
log(f'{tag}-stow-{stow}')
fire(stow)
t_end = time.time() + 2
while time.time() < t_end:
    msg = m.recv_match(type='SYS_STATUS', blocking=True, timeout=0.2)
    if msg:
        out.write(f'{time.time():.1f},{msg.voltage_battery/1000.0:.2f},stowed\n'); out.flush()
log(f'{tag}-done')
out.close()
print('投放时序完成')
