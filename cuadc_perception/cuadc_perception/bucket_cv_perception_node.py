#!/usr/bin/env python3
"""白桶 CV 感知节点（SSOT §4.1 副通道当主用，M2 视觉在环第一步）。

链路：gz-transport 直订 /d435i/image（RGB）+ ROS 订 /mavros/local_position/odom
（高度与取帧时间基准）→ LAB 白桶分割 + fitEllipse → 单目解算机体系 →
《接口契约.md》v1.0 PoseArray（/perception/drop_buckets_body）+ 心跳。

主通道（YOLOv8n-seg TensorRT，桶分割模型未训练）训练完成后在同类节点接入，
本节点的发布语义（stamp=最新 odom 戳−延迟、orientation 字段复用、空帧照发、
直径物理先验 0.08~0.35m 拒检）保持一致，届时按 IoU/置信度双通道融合。

运行注意（Jetson）：
  gz-transport13 Python 绑定需要纯 Python protobuf 解析——启动环境必须带
  PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python（fc_sitl_m2.sh 已带；
  本文件顶部也 setdefault 兜底）。
"""

import os
os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')

import math
import threading

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Header

from gz.transport13 import Node as GzNode
from gz.msgs10.image_pb2 import Image as GzImage

# 下视相机内参（iris_d435i_airframe/model.sdf：hfov=1.5rad 张在图像宽度上；
# 分辨率改了 fx/cx/cy 自动跟随，hfov 不变）
_GZ_IMAGE_TOPIC = '/d435i/image'
_HFOV_RAD = 1.5
_BUCKET_TOP_Z = 0.30      # 投放桶高 30cm（RULE_MAPPING.md）
_MIN_H_M = 0.30           # 解算高度下限（贴地/数据异常时兜底）
_D_DIAM_MIN_M = 0.08      # 契约物理先验（drop_logic 同款）
_D_DIAM_MAX_M = 0.35
_MAX_POSES = 8            # 契约单帧上限
_PIX_FORMAT_RGB_INT8 = 3


class BucketCvPerceptionNode(Node):

    def __init__(self):
        super().__init__('bucket_cv_perception')

        # ---- 参数 ----
        self.declare_parameter('pipeline_delay_s', 0.1)   # 取帧→发布人工延迟上界内
        self.declare_parameter('contract_version', 1.0)   # 契约哨兵
        self.declare_parameter('frame_id', 'cuadc_body_flu')
        self.declare_parameter('bucket_topic', '/perception/drop_buckets_body')
        self.declare_parameter('heartbeat_topic', '/perception/heartbeat')
        self.declare_parameter('l_min', 160.0)            # LAB L 通道白桶阈值（兜底下限）
        self.declare_parameter('l_offset', 10.0)          # 相对阈值：L > 帧中位数 + 偏移。
                                                          # 2026-09-12 排障：软件渲染把光照摊平
                                                          # （桶 L157 vs 场地 L143），固定阈值 160
                                                          # 差 1 灰度全灭；相对阈值两种渲染都成立
        self.declare_parameter('ab_max_dev', 22.0)        # A/B 离中性最大偏差（近白）
        self.declare_parameter('min_area_px', 40.0)       # 连通域最小面积
        self.declare_parameter('min_circularity', 0.60)   # 椭圆短/长轴比下限（下视圆）
        self.declare_parameter('confidence_base', 0.55)   # 置信度下限映射
        self.declare_parameter('debug_save_period', 0)    # 每 N 张存调试图（0=关）
        # 直径补偿：LAB 阈值+形态学腐蚀桶口边缘 → 椭圆偏小 ~15%（2026-09-12 标定
        # 实测：真值 0.25m 悬停读数 0.21~0.23）。补偿后落在 nominal±0.035 对号门内
        self.declare_parameter('diam_compensation', 1.15)

        gp = self.get_parameter
        self.delay_s = max(0.0, gp('pipeline_delay_s').value)
        self.version = gp('contract_version').value
        self.frame_id = gp('frame_id').value
        self.l_min = gp('l_min').value
        self.ab_max_dev = gp('ab_max_dev').value
        self.min_area = gp('min_area_px').value
        self.min_circ = gp('min_circularity').value
        self.conf_base = gp('confidence_base').value
        self.debug_period = int(gp('debug_save_period').value)
        self.diam_comp = max(1.0, gp('diam_compensation').value)
        self.l_offset = max(0.0, gp('l_offset').value)

        # ---- ROS 侧：odom（高度+时间基准）与发布 ----
        qos = qos_profile_sensor_data
        self.bucket_pub = self.create_publisher(
            PoseArray, gp('bucket_topic').value, qos)
        self.heartbeat_pub = self.create_publisher(
            Header, gp('heartbeat_topic').value, qos)
        self.create_subscription(
            Odometry, '/mavros/local_position/odom', self.on_odom, qos)
        self.odom_lock = threading.Lock()
        self.odom_stamp = None       # 最新 odom 戳（mavros/FCU 时间基准）
        self.oz = 0.0
        self.have_odom = False
        self.frame_count = 0
        self.detect_count = 0

        # ---- gz 侧：直订相机 ----
        self.gz_node = GzNode()
        n = self.gz_node.subscribe(GzImage, _GZ_IMAGE_TOPIC, self.on_image)
        if not n:
            raise RuntimeError(
                f'gz 订阅失败: {_GZ_IMAGE_TOPIC}（gz server 在跑？DISPLAY/EGL？）')

        self.get_logger().info(
            f'CV 白桶感知启动: gz({_GZ_IMAGE_TOPIC}) + odom 高度解算, '
            f'L>{self.l_min:.0f}/ab<{self.ab_max_dev:.0f}, '
            f'直径先验 {_D_DIAM_MIN_M:.2f}~{_D_DIAM_MAX_M:.2f}m, 延迟={self.delay_s:.2f}s')

    # ------------------------------------------------------------------
    def on_odom(self, msg):
        with self.odom_lock:
            self.odom_stamp = msg.header.stamp
            self.oz = msg.pose.pose.position.z
            self.have_odom = True

    def _intrinsics(self, w, h):
        fx = (w / 2.0) / math.tan(_HFOV_RAD / 2.0)
        return fx, w / 2.0, h / 2.0

    # ------------------------------------------------------------------
    def on_image(self, msg):
        if not self.have_odom:
            return                      # 无高度/无时间基准，首帧前不发
        if msg.pixel_format_type != _PIX_FORMAT_RGB_INT8:
            self.get_logger().warning(
                f'像素格式 {msg.pixel_format_type} 非 RGB8，拒帧', throttle_duration_sec=5.0)
            return
        w, h = msg.width, msg.height
        rgb = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(h, w, 3)
        frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        with self.odom_lock:
            stamp = self.odom_stamp
            oz = self.oz
        h_m = max(_MIN_H_M, oz - _BUCKET_TOP_Z)   # 相机离桶口高度
        fx, cx, cy = self._intrinsics(w, h)

        dets = self._detect(frame, oz, h_m, fx, cx, cy)

        # ---- 契约发布：stamp = 最新 odom 戳 − 延迟（与 P0.4 严格同源）----
        capture_time = None
        if stamp is not None:
            capture_time = rclpy.time.Time.from_msg(stamp) - \
                Duration(seconds=self.delay_s)
        msg_out = PoseArray()
        if capture_time is not None:
            msg_out.header.stamp = capture_time.to_msg()
        msg_out.header.frame_id = self.frame_id
        for (bx, by, bz, diam, conf) in dets:
            pose = Pose()
            pose.position.x = float(bx)
            pose.position.y = float(by)
            pose.position.z = float(bz)
            pose.orientation.x = float(diam)     # 复用：筒口直径（m）
            pose.orientation.y = float(conf)     # 复用：置信度 [0,1]
            pose.orientation.z = self.version    # 复用：契约版本哨兵
            pose.orientation.w = 0.0             # 保留
            msg_out.poses.append(pose)
        self.bucket_pub.publish(msg_out)

        hb = Header()
        if capture_time is not None:
            hb.stamp = capture_time.to_msg()
        hb.frame_id = 'bucket_cv_perception'
        self.heartbeat_pub.publish(hb)

        self.frame_count += 1
        if dets:
            self.detect_count += 1
        if self.frame_count % 20 == 0:
            self.get_logger().info(
                f'帧 {self.frame_count}, 检出帧 {self.detect_count}, '
                f'本帧 {len(dets)} 桶, oz={oz:.2f}m')
        if self.debug_period and dets and self.frame_count % self.debug_period == 0:
            for i, (bx, by, _, diam, conf) in enumerate(dets):
                u = int(cx + bx / h_m * fx)
                v = int(cy + by / h_m * fx)
                cv2.circle(frame, (u, v), 4, (0, 0, 255), 1)
                cv2.putText(frame, f'd={diam:.2f} c={conf:.2f}', (u + 6, v),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)
            cv2.imwrite(f'/tmp/cv_perception_{self.frame_count}.png', frame)

    # ------------------------------------------------------------------
    def _detect(self, frame_bgr, oz, h_m, fx, cx, cy):
        """LAB 白桶分割 + 椭圆拟合 + 单目解算。返回 [(x,y,z,diam,conf)]（机体系）。

        解算（下视，像素 (u,v)、光心 (cx,cy)、焦距 fx、离桶口高 h）：
          x = (u−cx)·h/fx，y = (v−cy)·h/fx，z = 桶口高 0.30 − odom 高（负=在下方）。
        """
        bz = _BUCKET_TOP_Z - oz
        lab = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2LAB)
        L, A, B = lab[:, :, 0].astype(np.float32), \
            lab[:, :, 1].astype(np.float32), lab[:, :, 2].astype(np.float32)
        # 相对阈值（抗渲染光照差异）+ 兜底下限：白桶 = 比全场中位数亮一截且色度中性
        l_thr = max(self.l_min, float(np.median(L)) + self.l_offset)
        mask = ((L > l_thr) &
                (np.abs(A - 128.0) < self.ab_max_dev) &
                (np.abs(B - 128.0) < self.ab_max_dev)).astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, (5, 5))

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        out = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area or len(cnt) < 5:
                continue
            (eu, ev), (d_a, d_b), _ang = cv2.fitEllipse(cnt)
            if d_a <= 1.0 or d_b <= 1.0:
                continue
            circularity = min(d_a, d_b) / max(d_a, d_b)
            if circularity < self.min_circ:
                continue
            diam_px = (d_a + d_b) / 2.0
            diam_m = diam_px / fx * h_m * self.diam_comp
            # 直径物理先验（免费一致性校验）：0.8m 起飞坪等大白块在此被拒
            if not (_D_DIAM_MIN_M <= diam_m <= _D_DIAM_MAX_M):
                continue
            # 置信度：圆度为主因子（仿真光照可控，不做光照异常降级）
            conf = min(0.95, self.conf_base + 0.45 * circularity)
            bx = (eu - cx) / fx * h_m          # 前向（图像宽方向=机体 x）
            by = (ev - cy) / fx * h_m          # 左向（图像高方向=机体 y）
            out.append((bx, by, bz, diam_m, conf))
            if len(out) >= _MAX_POSES:
                break
        return out


def main(args=None):
    rclpy.init(args=args)
    node = BucketCvPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
