#!/usr/bin/env python3
"""UVC 相机驱动节点：/dev/video0 → /camera/image_raw（替代未安装的 usb_cam 包）。

用法（Jetson）：
    nohup python3 ~/camera_node.py > /tmp/camera_node.log 2>&1 &

要点：
- 发布 QoS = RELIABLE depth2，与 hazard_recon 订阅端一致（09-12 实测：UDP-only
  下大图 best_effort 丢一个分片 = 丢整帧，RELIABLE 靠重传保整帧）；
- header.stamp = 取帧时刻（契约 §5.3），不是发布时刻；
- 曝光管理不在此节点：bench 用 v4l2-ctl 设 auto_exposure=3；飞行基线 §1.2
  手动锁快门——BL-500W-335 标准 UVC 手动曝光被固件无视（见 06 坑清单），待解。
"""
import subprocess
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image


class UsbCameraNode(Node):

    def __init__(self):
        super().__init__('usb_camera')
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('width', 1920)
        self.declare_parameter('height', 1080)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('topic', '/camera/image_raw')
        # 设备重开/驱动会话重置会把控制打回默认（AE 自动）——暗屋 AE 拉秒级长曝光，
        # 帧率塌到 1Hz。因此每次 open() 后强制重设本行控制。
        # manual(1)=固件固定短曝光；AE(3) 在室内会把快门拉满压垮帧率。
        # 飞行 §1.2 基线（手动锁快门 ≤1/500s）待固件问题解决后回填，见 06 坑清单。
        self.declare_parameter('v4l2_set_ctrls', 'auto_exposure=1,brightness=48')
        # 发布 CompressedImage(JPEG)：6MB 裸图 Image 在 rclpy+FastDDS 下单帧
        # 序列化/发送 ~1s（09-13 实测，全链被拖到 1Hz）；JPEG ~65KB/帧全链 30fps。
        # 压缩模式直接透传相机原生 MJPG 帧（CONVERT_RGB=0），零重编码、零二次压缩。
        self.declare_parameter('pub_compressed', True)
        # 发布 QoS：RELIABLE 会被慢速 RELIABLE 订阅端流控堵住（publish() 同步阻塞
        # → 全链被拖到消费速度）。bench 默认 best_effort 保帧率；飞行回 RELIABLE
        # 前必须先解决消费端异步化（见 06 坑清单 QoS 条目）。
        self.declare_parameter('pub_best_effort', True)
        gp = lambda n: self.get_parameter(n).value
        self.dev = gp('device')
        self.w, self.h, self.fps = gp('width'), gp('height'), gp('fps')
        self.ctrls = gp('v4l2_set_ctrls')
        self.compressed = gp('pub_compressed')

        rel = ReliabilityPolicy.BEST_EFFORT if gp('pub_best_effort') \
            else ReliabilityPolicy.RELIABLE
        qos = QoSProfile(depth=2, reliability=rel)
        topic = gp('topic')
        if self.compressed:
            self.pub = self.create_publisher(
                CompressedImage, topic + '/compressed', qos)
        else:
            self.pub = self.create_publisher(Image, topic, qos)
        self.cap = None
        self.n, self.t0 = 0, time.time()

    def open(self):
        # 控制必须写在开流之前：这颗 Realtek 固件在流激活中被 v4l2-ctl 写控制
        # 会掉进 ~1fps 怪模式（09-13 实测），且控制值跨会话持久，开流前写即生效。
        for kv in [s for s in self.ctrls.split(',') if s]:
            r = subprocess.run(
                ['v4l2-ctl', '-d', self.dev, '--set-ctrl', kv],
                capture_output=True, text=True)
            if r.returncode != 0:
                self.get_logger().warn(f'v4l2 set {kv} 失败: {r.stderr.strip()}')
        cap = cv2.VideoCapture(self.dev, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        if self.compressed:
            cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)  # 透传原生 JPEG，不解码
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.h)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        ok, frame = cap.read()
        if not ok:
            cap.release()
            raise RuntimeError(f'{self.dev} 打不开或读不到帧')
        self.get_logger().info(
            f'{self.dev} 就绪: {frame.shape[1]}x{frame.shape[0]} MJPG 目标{self.fps:.0f}fps '
            f'ctrls[{self.ctrls}]')
        self.cap = cap

    def spin(self):
        while rclpy.ok():
            if self.cap is None:
                try:
                    self.open()
                except Exception as e:
                    self.get_logger().error(f'{e}，2s 后重试')
                    time.sleep(2.0)
                    continue
            ok, frame = self.cap.read()
            if not ok:
                self.get_logger().error('读帧失败，重开设备')
                self.cap.release()
                self.cap = None
                continue
            stamp = self.get_clock().now().to_msg()  # 契约 §5.3：取帧时刻
            if self.compressed:
                # CONVERT_RGB=0 下 frame 形如 (1, N)：一行原生 MJPG 字节
                msg = CompressedImage()
                msg.header.stamp = stamp
                msg.header.frame_id = self.get_name()
                msg.format = 'jpeg'
                msg.data = frame.tobytes()
            else:
                msg = Image()
                msg.header.stamp = stamp
                msg.header.frame_id = self.get_name()
                msg.height, msg.width = frame.shape[:2]
                msg.encoding = 'bgr8'
                msg.step = frame.shape[1] * 3
                msg.data = frame.tobytes()
            self.pub.publish(msg)
            self.n += 1
            if self.n % 100 == 0:
                now = time.time()
                self.get_logger().info(f'已发 {self.n} 帧, {100.0 / (now - self.t0):.1f} FPS')
                self.t0 = now


def main():
    rclpy.init()
    node = UsbCameraNode()
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node.cap is not None:
            node.cap.release()
        try:
            node.destroy_node()
            rclpy.shutdown()
        except rclpy._rclpy_pybind11.RCLError:
            pass  # SIGTERM 下 context 可能已关，退出噪音不值一条 traceback


if __name__ == '__main__':
    main()
