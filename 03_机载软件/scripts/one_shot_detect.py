#!/usr/bin/env python3
"""单帧诊断：抓一帧 /camera/image_raw/compressed 跑 best.engine，打印检出框。
用于"窗口有画面但没识别框"时定性：模型链路通不通 vs 画面里本来就没有目标。
用法：python3 one_shot_detect.py  （跑完即退）
"""
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO

ENGINE = '/home/nvidia/cuadc_models/best.engine'

rclpy.init()
node = rclpy.create_node('one_shot_detect')
frames = []
node.create_subscription(CompressedImage, '/camera/image_raw/compressed',
                         lambda m: frames.append(m.data), qos_profile_sensor_data)
t0 = time.time()
while rclpy.ok() and not frames and time.time() - t0 < 10:
    rclpy.spin_once(node, timeout_sec=0.5)
assert frames, '10s 未收到相机帧——相机节点没起或 QoS 失配'
model = YOLO(ENGINE, task='detect')
# 黑帧预热 3 帧（部署铁律）
for _ in range(3):
    model.predict(np.zeros((640, 640, 3), dtype=np.uint8), imgsz=640, verbose=False)
frame = cv2.imdecode(np.frombuffer(frames[0], np.uint8), cv2.IMREAD_COLOR)
r = model.predict(frame, imgsz=640, conf=0.10, verbose=False)[0]
print(f'帧 {len(frames[0])}B {frame.shape[1]}x{frame.shape[0]}；检出 {len(r.boxes)} 框')
for b in r.boxes:
    print(f'  id{int(b.cls)} {r.names[int(b.cls)]} conf={float(b.conf):.2f} '
          f'xyxy={[round(v) for v in b.xyxy[0].tolist()]}')
node.destroy_node()
rclpy.shutdown()
