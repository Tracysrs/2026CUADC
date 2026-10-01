#!/usr/bin/env python3
"""单目标轮测:采 N 帧,双 engine 判,按类聚合中位置信。

用法: python3 test_one_target.py [帧数=6] [engineB=影子软链]
  engineA 固定=生产 ~/cuadc_models/best.engine(v2);engineB 默认 ~/v3_shadow/best_v3.engine
  (ln -sfn 切 v31/v32 后无需改本脚本)。engine 是进程启动时反序列化的——
  换软链对象后**必须重启本脚本**,旧进程不会跟着换(2026-10-01 live_ab 同坑实证)。
抓帧人眼验牌防张冠李戴(09-28「有毒品」轮教训):配合 live_ab 同屏或 kacha 存证。
"""
import sys, time, statistics
import cv2, numpy as np, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO

N = int(sys.argv[1]) if len(sys.argv) > 1 else 6
ENGB = sys.argv[2] if len(sys.argv) > 2 else "/home/nvidia/v3_shadow/best_v3.engine"
ENGINES = [("/home/nvidia/cuadc_models/best.engine", "v2"),
           (ENGB, "v3")]
names = {0:"爆炸品",1:"不燃气体",2:"刺激性",3:"放射性",4:"腐蚀品",5:"生物危害",6:"遇湿易燃",7:"有毒品",8:"自燃物品",9:"易燃",10:"投弹筒1",11:"投弹筒2",12:"投弹筒3",13:"landing_H"}

rclpy.init(); node = rclpy.create_node("t1t")
frames = []
node.create_subscription(CompressedImage, "/camera/image_raw/compressed",
    lambda m: frames.append(m.data), qos_profile_sensor_data)
t0 = time.time()
while rclpy.ok() and len(frames) < N and time.time() - t0 < 30:
    rclpy.spin_once(node, timeout_sec=0.5)
assert frames, "30s 未收到相机帧"
imgs = [cv2.imdecode(np.frombuffer(f, np.uint8), cv2.IMREAD_COLOR) for f in frames]
for eng, tag in ENGINES:
    m = YOLO(eng, task="detect")
    for _ in range(3):
        m.predict(np.zeros((640,640,3), dtype=np.uint8), imgsz=640, verbose=False)
    hits = {}
    for im in imgs:
        r = m.predict(im, imgsz=640, conf=0.10, verbose=False)[0]
        seen = set()
        for c, s in zip(r.boxes.cls.tolist(), r.boxes.conf.tolist()):
            k = int(c)
            if k in seen: continue
            seen.add(k); hits.setdefault(k, []).append(float(s))
    parts = [f"{names.get(k,k)} 中位{statistics.median(v):.2f}({len(v)}/{len(imgs)}帧)"
             for k, v in sorted(hits.items(), key=lambda x: -statistics.median(x[1])) if len(v) >= 3]
    print(f"{tag}: " + ("; ".join(parts) if parts else "无稳定检出"))
