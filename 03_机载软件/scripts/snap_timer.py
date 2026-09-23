#!/usr/bin/env python3
"""定时拍照：订阅 /camera/image_raw/compressed，每 interval 秒把最新帧存成 JPEG。
走 camera_node 的 ROS2 流，不抢 /dev/video0（与相机节点/识别管线并存，无查重问题）。
用法（在 Jetson 上）：
    python3 snap_timer.py                          # 默认 2s 一张，存 ~/photos
    python3 snap_timer.py --interval 5 --dir ~/photos_0923
    python3 snap_timer.py --detect                 # 顺带跑 best.engine 画框存图
    python3 snap_timer.py --detect --only-hit      # 只存有检出目标的帧（侦察留证）
Ctrl-C 结束。存储参考：1080p JPEG 约 0.2~0.4MB/张，2s 间隔 10 分钟 ≈ 300 张 60~120MB。
"""
import argparse
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

ENGINE = '/home/nvidia/cuadc_models/best.engine'


def main():
    ap = argparse.ArgumentParser(description='定时抓帧存图（订阅相机 ROS2 流，不抢设备）')
    ap.add_argument('--interval', type=float, default=2.0, help='拍照间隔秒（默认 2.0）')
    ap.add_argument('--dir', default=os.path.expanduser('~/photos'), help='保存目录（默认 ~/photos）')
    ap.add_argument('--detect', action='store_true', help='跑 best.engine 检测并画框存图')
    ap.add_argument('--only-hit', action='store_true', help='只存有检出的帧（需 --detect）')
    ap.add_argument('--conf', type=float, default=0.10, help='检测置信度阈值（默认 0.10）')
    args = ap.parse_args()
    args.dir = os.path.expanduser(args.dir)
    os.makedirs(args.dir, exist_ok=True)

    model = None
    if args.detect:
        from ultralytics import YOLO
        model = YOLO(ENGINE, task='detect')
        for _ in range(3):  # 黑帧预热（部署铁律）
            model.predict(np.zeros((640, 640, 3), dtype=np.uint8), imgsz=640, verbose=False)

    rclpy.init()
    node = rclpy.create_node('snap_timer')
    latest = {'data': None}
    node.create_subscription(CompressedImage, '/camera/image_raw/compressed',
                             lambda m: latest.__setitem__('data', m.data),
                             qos_profile_sensor_data)

    mode = '检测画框' if args.detect else '纯存图'
    extra = '，只存命中帧' if (args.detect and args.only_hit) else ''
    print(f"定时拍照：间隔 {args.interval}s、{mode}{extra} -> {args.dir}；Ctrl-C 结束")

    n_saved = 0
    last_t = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.time()
            if latest['data'] is None or now - last_t < args.interval:
                continue
            last_t = now
            frame = cv2.imdecode(np.frombuffer(latest['data'], np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                continue
            hits = 0
            if model is not None:
                r = model.predict(frame, imgsz=640, conf=args.conf, verbose=False)[0]
                hits = len(r.boxes)
                for b in r.boxes:
                    x1, y1, x2, y2 = [int(v) for v in b.xyxy[0].tolist()]
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 200, 0), 2)
                    cv2.putText(frame, f"{r.names[int(b.cls)]} {float(b.conf):.2f}",
                                (x1, max(20, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 0), 2)
            if args.only_hit and hits == 0:
                continue
            path = os.path.join(args.dir, f"snap_{time.strftime('%Y%m%d_%H%M%S')}.jpg")
            cv2.imwrite(path, frame)
            n_saved += 1
            print(f"[{n_saved}] {path}（{hits} 框）")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print(f"结束，共存 {n_saved} 张 -> {args.dir}")


if __name__ == '__main__':
    main()
