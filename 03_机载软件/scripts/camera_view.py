#!/usr/bin/env python3
"""订阅 /camera/image_raw/compressed 实时显示（接在 Jetson 机身显示器上调试用）。

用法（SSH 里启动，窗口显示在本机屏幕）：
    DISPLAY=:0 nohup python3 ~/camera_view.py > /tmp/camera_view.log 2>&1 &
按 q 退出。

订阅 QoS 用 best_effort（查看器绝不反压识别链路）。
"""
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class CameraView(Node):

    def __init__(self):
        super().__init__('camera_view')
        self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed', self.on_image,
            qos_profile_sensor_data)
        self.n, self.t0, self.fps = 0, time.time(), 0.0

    def on_image(self, msg):
        buf = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        self.n += 1
        now = time.time()
        if now - self.t0 >= 1.0:
            self.fps = self.n / (now - self.t0)
            self.n, self.t0 = 0, now
        cv2.putText(frame, f'{frame.shape[1]}x{frame.shape[0]}  {self.fps:.1f} FPS  (q quit)',
                    (16, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        cv2.imshow('cuadc camera', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            raise KeyboardInterrupt


def main():
    rclpy.init()
    node = CameraView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
