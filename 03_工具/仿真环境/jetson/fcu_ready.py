#!/usr/bin/env python3
"""FCU 就绪门禁 v2：双端口探活 + GPS 就绪校验 + 耦合僵死自动自愈。

为什么存在：reset 后有分钟级"启动不稳定窗"（gz→SITL 传感器耦合逐步建立），
且长时间空转后耦合会僵死（GPS 归零但心跳/姿态仍在流——2026-09-13 实证），
旧版门禁只探心跳流会误放行，任务卡死在 WAIT_NAV_STABLE。

v2 三层判据：
  1. 连接：轮询 5762 与 5760（5762 是 SERIAL1，boot 后段才绑定；5760 是 SERIAL0
     单客户端——手动跑门禁时 mavros 未启动、5760 空闲，直连最快；任务脚本内
     mavros 已占 5760 时自动只用 5762）；
  2. 心跳稳定窗：任意 MAVLink 消息连续 --window 秒不断档（同 v1）；
  3. **GPS 就绪**：fix_type ≥ 3D 才算真就绪（心跳流≠耦合健康）。

自动自愈（--auto-heal，默认开）：GPS 卡 0 超 --gps-timeout 秒且 SITL 进程在，
自动重启 gz 服务端（FDM 重新握手，SITL 不动不进长窗）并继续探——把"玄学等
20 分钟"变成 ~1 分钟自愈。--no-heal 关闭。

用法：
  python3 fcu_ready.py                       # 默认 30s 稳定窗 / 600s 超时 / 自愈开
  python3 fcu_ready.py --window 10 --timeout 60
退出码：0 = FCU 就绪；1 = 超时未就绪。
"""
import argparse
import subprocess
import sys
import time

from pymavlink import mavutil


def try_connect(port, timeout):
    """连一端口并等心跳。成功返回连接对象，否则 None。"""
    try:
        m = mavutil.mavlink_connection(
            'tcp:127.0.0.1:%d' % port, source_system=250, retries=1)
        hb = m.wait_heartbeat(timeout=timeout)
        if hb is None:
            m.close()
            return None
        return m
    except Exception:
        return None


def stability_window(m, window, gap):
    """任意 MAVLink 消息连续 window 秒不断档（sketch 重启时流会断，靠 gap 兜底）。"""
    first = None
    last = time.time()
    while True:
        msg = m.recv_match(blocking=True, timeout=gap)
        now = time.time()
        if msg is None or now - last > gap:
            return False
        last = now
        if first is None:
            first = now
        if now - first >= window:
            return True


def gps_ready(m, timeout, on_tick=None):
    """等 GPS fix_type ≥ 3（EKF 有位置源才算真就绪）。"""
    m.mav.request_data_stream_send(
        m.target_system, m.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2)
        if msg is not None and msg.fix_type >= 3:
            return True
        if on_tick:
            on_tick()
    return False


def restart_gz():
    """重启 gz 服务端（FDM 重新握手；SITL 不动）。"""
    print('  [自愈] 重启 gz 服务端（FDM 重新握手，SITL 不动）...', flush=True)
    subprocess.run(['pkill', '-f', 'gz [s]im -s'], check=False)
    time.sleep(3)
    subprocess.run(['bash', '/home/nvidia/sim_scripts/start_gz.sh'],
                   check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(30)


def main():
    ap = argparse.ArgumentParser(description='SITL FCU 就绪门禁 v2（双端口+GPS 校验+自愈）')
    ap.add_argument('--ports', default='5762,5760',
                    help='轮询端口（5760=SERIAL0 单客户端，mavros 在跑时自动跳过）')
    ap.add_argument('--window', type=float, default=30.0, help='心跳稳定窗（s）')
    ap.add_argument('--timeout', type=float, default=600.0, help='总超时（s）')
    ap.add_argument('--gap', type=float, default=3.0, help='心跳最大间隔（s）')
    ap.add_argument('--gps-timeout', type=float, default=90.0,
                    help='GPS fix 等 3D 的时间上限（s），超时触发自愈/失败')
    ap.add_argument('--auto-heal', dest='auto_heal', action='store_true', default=True,
                    help='GPS 卡 0 自动重启 gz（默认开）')
    ap.add_argument('--no-heal', dest='auto_heal', action='store_false', help='关闭自愈')
    args = ap.parse_args()

    ports = [int(p) for p in args.ports.split(',')]
    deadline = time.time() + args.timeout
    attempt = 0
    heals = 0
    while time.time() < deadline:
        attempt += 1
        m = None
        for port in ports:
            m = try_connect(port, timeout=6)
            if m is not None:
                print('  [%s] 端口 %d 心跳出现（mode=%s），开始 %.0fs 稳定窗'
                      % (time.strftime('%H:%M:%S'), port, m.flightmode, args.window),
                      flush=True)
                break
        if m is None:
            print('  [%s] 第%d次: 5762/5760 均未就绪（%s 后重试）'
                  % (time.strftime('%H:%M:%S'), attempt,
                     'SITL 未在跑，先 reset_sim.sh' if attempt % 6 == 0 else '不稳定窗/等'),
                  flush=True)
            time.sleep(5)
            continue

        if not stability_window(m, args.window, args.gap):
            print('  [%s] 断档（不稳定窗未过），重新计窗' % time.strftime('%H:%M:%S'),
                  flush=True)
            m.close()
            time.sleep(2)
            continue

        # 心跳稳了 → 校验 GPS（心跳流≠耦合健康）
        heal_note = ['']

        def tick():
            if time.time() > deadline - 5:
                return
            heal_note[0] = ''

        ok = gps_ready(m, args.gps_timeout)
        if ok:
            print('  [%s] FCU 就绪：稳定窗 %.0fs + GPS 3D ✓'
                  % (time.strftime('%H:%M:%S'), args.window), flush=True)
            m.close()
            return 0
        if args.auto_heal and heals < 4:
            heals += 1
            print('  [%s] 心跳在流但 GPS 卡 0（耦合僵死），自愈第 %d 次'
                  % (time.strftime('%H:%M:%S'), heals), flush=True)
            m.close()
            restart_gz()
            continue
        print('  [%s] GPS 未达 3D（自愈%s）' % (time.strftime('%H:%M:%S'),
              '次数用尽' if args.auto_heal else '已关闭'), flush=True)
        m.close()
        time.sleep(5)

    print('超时（%.0fs）未就绪 → 勿开跑任务' % args.timeout, flush=True)
    return 1


if __name__ == '__main__':
    sys.exit(main())
