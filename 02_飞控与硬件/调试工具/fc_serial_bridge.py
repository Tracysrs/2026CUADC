# -*- coding: utf-8 -*-
"""FC 串口 <-> 本机网络桥：Mission Planner 半死连接（开得了口收不到数据）时的绕接通道。

场景（2026-10-03 实战）：FC 侧实测满速流送正常，MP 打开 COM 口却收不到任何数据，
重插重连不愈——把 FC 串口桥到本机 TCP/UDP，MP 换网络方式连接即通，链路诊断不受影响。
  TCP 模式（推荐，MP 弹窗就两个填空框）：python fc_serial_bridge.py
    MP：口位选 TCP -> host 127.0.0.1 -> 端口 14550
  UDP 模式：python fc_serial_bridge.py --mode udp
    MP：口位选 UDP -> 弹窗端口填 14550
桥自身周期发 MAVLink1 GCS 心跳保流送；Ctrl+C 停止。

坑注：pymavlink（Python 3.14 实测）mavlink_connection() 不吃 pyserial 对象
（device.startswith 崩）且把 "COMx" 误分发给文件读取——GCS 心跳用 io.BytesIO
预生成 MAVLink1 包裸写串口，不经 mavlink_connection。
口位陷阱：COM6=SLCAN（打开了也是哑巴）、数传=CP210x，认准描述 "ArduPilot MAVLink"。
"""
import argparse
import io
import socket
import serial
import threading
import time

from pymavlink import mavutil


def pregen_heartbeat():
    buf = io.BytesIO()
    mav = mavutil.mavlink.MAVLink(buf)
    mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                       mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    return buf.getvalue()


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--serial", default="COM5", help="FC 串口（默认 COM5）")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--mode", choices=["tcp", "udp"], default="tcp")
    ap.add_argument("--host", default="127.0.0.1", help="对端地址（默认本机）")
    ap.add_argument("--net-port", type=int, default=14550)
    args = ap.parse_args()

    ser = serial.Serial(args.serial, args.baud, timeout=0.1)
    hb = pregen_heartbeat()
    print(f"bridge: {args.serial}@{args.baud} <-> {args.mode}:{args.host}:{args.net_port}，Ctrl+C 停",
          flush=True)

    state = {"client": None}

    def serial_to_net():
        n = 0
        while True:
            data = ser.read(512)
            if not data:
                continue
            if args.mode == "tcp":
                c = state["client"]
                if c is None:
                    continue
                try:
                    c.sendall(data)
                except Exception:
                    pass
            else:
                try:
                    s.sendto(data, (args.host, args.net_port))
                except OSError:
                    pass  # 对端未监听时 Windows 会连续抛 10054，MP 连上即消
            n += len(data)
            if n >= 4096:
                print(f"  FC->MP 流通中 累计 {n} 字节", flush=True)
                n = 0

    def net_to_serial():
        while True:
            if args.mode == "tcp":
                conn, addr = srv.accept()
                print("MP 已接入:", addr, flush=True)
                state["client"] = conn
                conn.settimeout(1.0)
                try:
                    while True:
                        try:
                            data = conn.recv(2048)
                            if not data:
                                break
                            ser.write(data)
                        except socket.timeout:
                            pass
                except Exception as e:
                    print("client err:", e, flush=True)
                print("MP 断开，等待重接", flush=True)
                state["client"] = None
            else:
                s.settimeout(1.0)
                try:
                    data, _ = s.recvfrom(2048)
                except socket.timeout:
                    continue
                except OSError:
                    continue
                ser.write(data)

    if args.mode == "tcp":
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((args.host, args.net_port))
        srv.listen(1)
    else:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    threading.Thread(target=serial_to_net, daemon=True).start()
    threading.Thread(target=net_to_serial, daemon=True).start()

    try:
        while True:
            ser.write(hb)
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nbridge stopped")


if __name__ == "__main__":
    main()
