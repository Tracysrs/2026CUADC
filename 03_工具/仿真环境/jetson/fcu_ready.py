#!/usr/bin/env python3
"""FCU 就绪门禁：直连 SITL SERIAL1(TCP 5762) 探心跳稳定窗。

为什么存在：reset_sim 之后有分钟级"启动不稳定窗"——sketch 反复重启（mavros 侧
表现为 STATUSTEXT "PreArm: Accels inconsistent" + TM: Time jump detected），
期间任务跑起来必然 armed:false mode:'' 全程空转。等心跳连续稳定 window 秒再开跑，
整体跳过这个窗（实测自愈，不必反复 reset——每次 reset 反而重新进窗）。

为什么探 5762 不探 5760：SERIAL0(5760) 是单客户端，跑门禁时 mavros 已占着它；
5762(SERIAL1) 是同一 FCU 的第二个 TCP 出口（-I0 默认 5760/5762/5763），
boot 中后段才 bind，探针自带重试即可。

为什么不用 ros2 topic echo 探：DDS 发现慢，--once 假阴性（09-10 老坑），
探 mavros 永远不可信；直连 pymavlink 才是确定性判据。

用法：
  python3 fcu_ready.py                        # 30s 稳定窗 / 600s 超时（默认）
  python3 fcu_ready.py --window 10 --timeout 60   # 栈已稳时的快速确认
退出码：0 = FCU 就绪；1 = 超时未稳定。
"""
import argparse
import sys
import time

from pymavlink import mavutil


def main():
    ap = argparse.ArgumentParser(description="SITL FCU 就绪门禁（心跳稳定窗）")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5762, help="SERIAL1 TCP 口（5760 留给 mavros）")
    ap.add_argument("--window", type=float, default=30.0, help="要求的连续稳定时长（s）")
    ap.add_argument("--timeout", type=float, default=600.0, help="总超时（s）；reset 后不稳定窗实测可达 ~10min")
    ap.add_argument("--gap", type=float, default=3.0, help="心跳最大允许间隔（s），超过视为断档重新计窗")
    args = ap.parse_args()

    deadline = time.time() + args.timeout
    attempt = 0
    while time.time() < deadline:
        attempt += 1
        try:
            m = mavutil.mavlink_connection(
                "tcp:%s:%d" % (args.host, args.port), source_system=250, retries=1)
            hb = m.wait_heartbeat(timeout=20)
            if hb is None:
                print("  [%s] 第%d次: TCP 连上但 20s 无心跳（boot 未完成，等）"
                      % (time.strftime("%H:%M:%S"), attempt), flush=True)
                m.close()
                time.sleep(5)
                continue
            print("  [%s] 心跳出现 mode=%s，开始 %.0fs 稳定窗"
                  % (time.strftime("%H:%M:%S"), m.flightmode, args.window), flush=True)
            first = None
            last = time.time()
            stable = False
            while not stable:
                msg = m.recv_match(blocking=True, timeout=args.gap)
                now = time.time()
                if msg is None or now - last > args.gap:
                    print("  [%s] 断档 %.1fs（不稳定窗未过），重新计窗"
                          % (time.strftime("%H:%M:%S"), now - last), flush=True)
                    break
                # 任何 MAVLink 消息都算活性：sketch 重启时流会断，靠 gap 兜底
                last = now
                if first is None:
                    first = now
                if now - first >= args.window:
                    stable = True
            if stable:
                print("  [%s] FCU 就绪：心跳连续 %.0fs（mode=%s）"
                      % (time.strftime("%H:%M:%S"), args.window, m.flightmode), flush=True)
                m.close()
                return 0
            m.close()
        except Exception as e:
            print("  [%s] 第%d次连接失败: %s（5s 后重试）"
                  % (time.strftime("%H:%M:%S"), attempt, e), flush=True)
            time.sleep(5)
    print("超时（%.0fs）未达成 %.0fs 稳定窗 → FCU 未就绪，勿开跑任务"
          % (args.timeout, args.window), flush=True)
    return 1


if __name__ == "__main__":
    sys.exit(main())
