#!/usr/bin/env python3
"""live_ab · v2|v3 同屏实时判读台(2026-09-30 爆炸品直采回流终判用)。

浏览器打开 http://<jetson-ip>:8080 ——上屏=现役 v2(生产 best.engine),下屏=v3 影子
(~/v3_shadow/best_v3.engine),同帧各自画框,类别=英文缩写+conf。纯订阅相机流,
零产线写入;判读完 pkill -f live_ab.py 释放 GPU。
用法: nohup python3 ~/v3_shadow/live_ab.py > /tmp/live_ab.log 2>&1 &
"""
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

ENGS = {"v2": "/home/nvidia/cuadc_models/best.engine",
        "v3": "/home/nvidia/v3_shadow/best_v3.engine"}
SHORT = {0: "EXPL", 1: "NGAS", 2: "IRRIT", 3: "RADIO", 4: "CORR",
         5: "BIO", 6: "WREACT", 7: "TOXIC", 8: "SPONT", 9: "FLAM"}
CONF = 0.45


def draw(img, m, tag):
    r = m.predict(img, imgsz=640, conf=CONF, verbose=False)[0]
    for b in r.boxes:
        x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())
        c, cf = int(b.cls[0]), float(b.conf[0])
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(img, f"{c} {SHORT.get(c, '?')} {cf:.2f}", (x1, max(22, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.putText(img, tag, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 255, 255), 3)
    return img


def main():
    from ultralytics import YOLO
    models = {k: YOLO(v, task="detect") for k, v in ENGS.items()}
    for m in models.values():
        for _ in range(3):
            m.predict(np.zeros((640, 640, 3), np.uint8), imgsz=640, verbose=False)
    print("engines warm", flush=True)

    rclpy.init()
    node = rclpy.create_node("live_ab")
    latest = {"jpg": None}
    lock = threading.Lock()

    def cb(msg):
        with lock:
            latest["jpg"] = msg.data

    node.create_subscription(CompressedImage, "/camera/image_raw/compressed",
                             cb, qos_profile_sensor_data)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    class H(BaseHTTPRequestHandler):
        def do_GET(self):
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            n = 0
            while True:
                with lock:
                    jpg = latest["jpg"]
                if jpg is None:
                    time.sleep(0.1)
                    continue
                img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                if img is None:
                    time.sleep(0.05)
                    continue
                top = draw(img.copy(), models["v2"], "v2 (production)")
                bot = draw(img, models["v3"], "v3 (shadow)")
                canvas = np.vstack([top, bot])
                ok, enc = cv2.imencode(".jpg", canvas, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if not ok:
                    continue
                try:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                                     + enc.tobytes() + b"\r\n")
                except Exception:
                    break
                n += 1
                time.sleep(0.06)  # ~10fps 判读足够

        def log_message(self, *a):
            pass

    srv = ThreadingHTTPServer(("0.0.0.0", 8080), H)
    print("serving :8080", flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
