#!/usr/bin/env python3
"""H 圆感知与精准降落节点 — 检测起降区 H 圆 → MAVROS LANDING_TARGET → ArduPilot PLND。

链路：/camera/image_raw/compressed → odom 插值取帧高度 → 检测（后端可配
yolo | hough | both，engine 缺失自动 Hough 兜底）→ 质量门控（直径-高度一致性 +
连续帧确认）→ /mavros/landing_target/set（mavros_msgs/LandingTarget）
+ 心跳 /perception/heartbeat_land（契约 v1.3 §4）。

飞控侧（SSOT §4.5）：PLND_ENABLED=1、PLND_TYPE=1（1=CompanionComputer MAVLink；
0=Never、2=IRLOCK——04_算法模块 §11 旧文把 0/1 写反，2026-09-13 勘误）。
质量门控不过就**不发**目标：PLND 无目标 = 自然普通降落，fail-safe 天然成立；
mission_node LAND 态零改动（/mavros/cmd/land → LAND 模式 → PLND 自动接管）。

坐标系陷阱（设计文档 §6.3）：契约机体系 = FLU（y 左 z 上），MAVLink BODY_FRD =
（y 右 z 下）——发 LANDING_TARGET 前必须 flu_to_frd（y、z 取反）。
上机待验证项（设计文档 §8）：mavros_msgs/LandingTarget 字段以 Jetson
`ros2 interface show mavros_msgs/msg/LandingTarget`（MAVROS 2.14）为准，代码对
非核心字段做了 hasattr 防御；landing_target 插件须在 apm.launch 插件列表中。
"""

import math
import os
import threading
import time
from collections import deque

import cv2
import numpy as np

from mavros_msgs.msg import LandingTarget
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header, String

from cuadc_perception import vision_core as vc

_HFOV_RAD = 1.4279
_VFOV_RAD = 1.1519
# MAV_FRAME 枚举（MAVLink common）：8=BODY_NED（轴序与 FRD 相同）、12=BODY_FRD
_MAV_FRAME_BODY_NED = 8
_MAV_FRAME_BODY_FRD = 12
_MAV_LANDING_TARGET_TYPE_VISION_FIDUCIAL = 1


def _stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class HCircleNode(Node):

    def __init__(self):
        super().__init__('h_circle_perception')
        gp = self.get_parameter

        # ---- 图像 / 高度（与桶节点同源约定）----
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('image_compressed', True)
        self.declare_parameter('image_qos_reliable', False)
        self.declare_parameter('odom_topic', '/mavros/local_position/odom')
        self.declare_parameter('calib_path', '/home/nvidia/cuadc_models/camera_calib.yaml')
        self.declare_parameter('allow_uncalibrated', True)
        self.declare_parameter('mount_rot_deg', 0.0)
        self.declare_parameter('image_width', 1920)
        self.declare_parameter('image_height', 1080)
        self.declare_parameter('ground_z_offset', 0.0)   # H 圆贴地 → plane_z = 0
        self.declare_parameter('odom_max_delay_s', 1.5)
        self.declare_parameter('odom_future_tol_s', 0.05)
        self.declare_parameter('odom_max_gap_s', 0.2)
        self.declare_parameter('odom_history_span_s', 3.0)

        # ---- 检测后端 ----
        self.declare_parameter('backend', 'both')        # yolo | hough | both
        self.declare_parameter('engine_path', '/home/nvidia/cuadc_models/h_circle.engine')
        self.declare_parameter('expected_sha256', '')
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('det_conf_thres', 0.25)
        for name, default in [('hough_radius_tol', 0.35), ('hough_param1', 120.0),
                              ('hough_param2', 40.0), ('hough_h_score_min', 0.45)]:
            self.declare_parameter(name, default)

        # ---- 质量门控（设计文档 §6.2）----
        self.declare_parameter('h_diam_m', vc.H_CIRCLE_DIAM_M)
        self.declare_parameter('h_diam_tol_m', 0.25)     # 直径-高度一致性容差
        self.declare_parameter('confirm_frames', 3)      # 连续帧确认后才首发
        self.declare_parameter('fuse_iou_thr', 0.5)

        # ---- LANDING_TARGET 发布 ----
        self.declare_parameter('landing_target_topic', '/mavros/landing_target/set')
        self.declare_parameter('lt_frame', 'BODY_FRD')   # BODY_FRD(12) | BODY_NED(8)
        self.declare_parameter('max_rate_hz', 20.0)      # ≥10Hz（PLND 要求），30Hz 帧率下降额
        self.declare_parameter('heartbeat_topic', '/perception/heartbeat_land')
        self.declare_parameter('debug_pose_topic', '/perception/h_circle_body')
        self.declare_parameter('mission_state_topic', '/cuadc/mission_state')
        self.declare_parameter('active_states', '')      # 空=全态；正式飞行可设 'RETURN_HOME,LAND'
        self.declare_parameter('debug_save_period', 0)

        # ---- 参数读入 ----
        self.backend = str(gp('backend').value).lower()
        self.mount_rot = float(gp('mount_rot_deg').value)
        self.ground_z = float(gp('ground_z_offset').value)
        self.max_delay = float(gp('odom_max_delay_s').value)
        self.future_tol = float(gp('odom_future_tol_s').value)
        self.max_gap = float(gp('odom_max_gap_s').value)
        self.h_diam = float(gp('h_diam_m').value)
        self.h_diam_tol = float(gp('h_diam_tol_m').value)
        self.confirm_frames = max(1, int(gp('confirm_frames').value))
        self.lt_frame = _MAV_FRAME_BODY_FRD if str(gp('lt_frame').value) == 'BODY_FRD' \
            else _MAV_FRAME_BODY_NED
        self.min_interval = 1.0 / max(1.0, float(gp('max_rate_hz').value))
        self.debug_period = int(gp('debug_save_period').value)
        state_str = str(gp('active_states').value).strip()
        self.active_states = [s.strip() for s in state_str.split(',') if s.strip()] \
            if state_str else []
        self.img_w = int(gp('image_width').value)
        self.img_h = int(gp('image_height').value)

        calib_path = os.path.expanduser(str(gp('calib_path').value))
        self.intr, warns = vc.load_intrinsics(
            calib_path, self.img_w, self.img_h,
            hfov_rad=_HFOV_RAD, vfov_rad=_VFOV_RAD,
            allow_uncalibrated=bool(gp('allow_uncalibrated').value))
        for w in warns:
            self.get_logger().warn(w)
        self.get_logger().info(
            f'内参[{self.intr.source}]: fx={self.intr.fx:.1f} fy={self.intr.fy:.1f}')

        self.hough_params = vc.HoughParams(
            radius_tol=float(gp('hough_radius_tol').value),
            param1=float(gp('hough_param1').value),
            param2=float(gp('hough_param2').value),
            h_score_min=float(gp('hough_h_score_min').value))

        self.model = None
        if self.backend in ('yolo', 'both'):
            self.model = self._load_engine_optional(
                os.path.expanduser(str(gp('engine_path').value)),
                str(gp('expected_sha256').value).strip(),
                int(gp('imgsz').value), float(gp('det_conf_thres').value))
            if self.model is None and self.backend == 'yolo':
                self.get_logger().error('backend=yolo 但 engine 不可用 → 改用 hough 兜底')
                self.backend = 'hough'

        # ---- odom 历史 ----
        self.odom_buf = deque()
        self.odom_lock = threading.Lock()
        self.span_s = float(gp('odom_history_span_s').value)

        # ---- 发布 / 订阅 ----
        qos = qos_profile_sensor_data
        self.lt_pub = self.create_publisher(
            LandingTarget, gp('landing_target_topic').value, 10)
        self.hb_pub = self.create_publisher(
            Header, gp('heartbeat_topic').value, qos)
        self.dbg_pub = self.create_publisher(
            PoseStamped, gp('debug_pose_topic').value, qos)
        self.create_subscription(Odometry, gp('odom_topic').value, self.on_odom, qos)
        self.create_subscription(String, gp('mission_state_topic').value,
                                 self.on_state, 10)
        self.image_compressed = bool(gp('image_compressed').value)
        if self.image_compressed:
            self.create_subscription(
                CompressedImage, gp('image_topic').value + '/compressed',
                self.on_image, qos)
        else:
            self.create_subscription(
                Image, gp('image_topic').value, self.on_image, qos)

        self.current_state = ''
        self.confirm_count = 0
        self.last_lt_send = 0.0
        self.lt_sent = 0
        self.frame_count = 0
        self.last_report = time.time()
        self.get_logger().info(
            f'H 圆感知就绪: backend={self.backend}, 门控 直径 {self.h_diam}±{self.h_diam_tol}m, '
            f'确认 {self.confirm_frames} 帧, frame={gp("lt_frame").value}, '
            f'active_states={self.active_states or "全态"}')

    # ---------------- engine（可选加载，失败自动 Hough） ----------------
    def _load_engine_optional(self, engine_path, expected_sha, imgsz, conf):
        self.imgsz, self.det_conf = imgsz, conf
        if not os.path.isfile(engine_path):
            self.get_logger().warn(
                f'h_marker engine 不存在({engine_path})——Hough 通道独立可用，'
                f'模型训练部署后自动升级（设计文档 §6.1）')
            return None
        if expected_sha:
            got = vc.sha256_of_file(engine_path)
            if got != expected_sha:
                self.get_logger().error(
                    f'engine SHA-256 不符！拒用主通道，保持 Hough 兜底 '
                    f'(expected={expected_sha} got={got})')
                return None
        else:
            self.get_logger().warn('未配置 expected_sha256，跳过主通道校验（仅限调试）')
        try:
            from ultralytics import YOLO
            t0 = time.time()
            model = YOLO(engine_path, task='detect')
            black = np.zeros((imgsz, imgsz, 3), dtype=np.uint8)
            for _ in range(3):
                model.predict(black, imgsz=imgsz, verbose=False)
            self.get_logger().info(
                f'h_marker engine 就绪 (加载+预热 {time.time()-t0:.1f}s)')
            return model
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'engine 加载失败({e})，保持 Hough 兜底')
            return None

    # ---------------- 订阅 ----------------
    def on_state(self, msg: String):
        self.current_state = msg.data

    def on_odom(self, msg: Odometry):
        t = _stamp_to_sec(msg.header.stamp)
        with self.odom_lock:
            self.odom_buf.append((t, float(msg.pose.pose.position.z)))
            while self.odom_buf and t - self.odom_buf[0][0] > self.span_s:
                self.odom_buf.popleft()

    def _odom_z_at(self, t: float):
        with self.odom_lock:
            buf = list(self.odom_buf)
        if not buf:
            return None
        newest_t, newest_z = buf[-1]
        if t > newest_t + self.future_tol or newest_t - t > self.max_delay:
            return None
        if t >= newest_t:
            return newest_z
        oldest_t, oldest_z = buf[0]
        if t <= oldest_t:
            return oldest_z
        for i in range(len(buf) - 2, -1, -1):
            t0, z0 = buf[i]
            t1, z1 = buf[i + 1]
            if t0 <= t <= t1:
                if t1 - t0 > self.max_gap:
                    return None
                r = (t - t0) / max(1e-9, t1 - t0)
                return z0 + r * (z1 - z0)
        return None

    def on_image(self, msg):
        try:
            self._process(msg)
        except Exception as e:  # noqa: BLE001 —— 单帧异常不丢节点，心跳照发
            self.get_logger().error(f'帧处理异常: {e}')
            self._publish_heartbeat(_stamp_to_sec(msg.header.stamp))

    # ---------------- 主链路 ----------------
    def _process(self, msg):
        if self.image_compressed:
            frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                raise ValueError('JPEG 解码失败（截断帧？）')
        else:
            frame = np.frombuffer(msg.data, np.uint8).reshape(
                msg.height, msg.width, 3)
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()
        t_frame = _stamp_to_sec(stamp)

        self._publish_heartbeat(t_frame)
        self.frame_count += 1

        if self.active_states and self.current_state not in self.active_states:
            self.confirm_count = 0
            return

        fh, fw = frame.shape[:2]
        intr = self.intr if (fw, fh) == (self.intr.width, self.intr.height) \
            else self.intr.scaled_to(fw, fh)

        oz = self._odom_z_at(t_frame)
        if oz is None:
            self.get_logger().warn('取帧时刻无可用 odom 高度，本帧不产目标',
                                   throttle_duration_sec=5.0)
            self.confirm_count = 0
            return
        h_m = max(0.10, oz - self.ground_z)          # H 圆贴地：平面 z=0

        dets = self._detect(frame, h_m, intr)
        target = max(dets, key=lambda d: d.conf) if dets else None

        # 质量门控①：直径-高度一致性（Hough 半径窗已隐含，YOLO 框必须复检）
        if target is not None:
            _, _, _, diam = vc.ellipse_to_body(
                target.u, target.v, target.a_px, target.b_px, intr, h_m, 0.0,
                self.mount_rot)
            if abs(diam - self.h_diam) > self.h_diam_tol:
                self.get_logger().debug(
                    f'直径-高度不符: {diam:.2f}m @h={h_m:.2f}m，拒')
                target = None

        # 质量门控②：连续帧确认（防单帧误检直接驱动 PLND）
        if target is None:
            self.confirm_count = 0
        else:
            self.confirm_count += 1

        confirmed = target is not None and self.confirm_count >= self.confirm_frames

        if target is not None:
            x, y, z, diam = vc.ellipse_to_body(
                target.u, target.v, target.a_px, target.b_px, intr, h_m, 0.0,
                self.mount_rot)
            self._publish_debug(x, y, z, diam, t_frame)
            if confirmed:
                self._publish_landing_target(x, y, h_m, diam, t_frame)

        now = time.time()
        if now - self.last_report >= 10.0:
            rate = self.frame_count / max(1e-9, now - self.last_report)
            self.last_report = now
            self.get_logger().info(
                f'{rate:.1f}FPS, 帧数 {self.frame_count}, 检出 {len(dets)}, '
                f'确认计数 {self.confirm_count}/{self.confirm_frames}, '
                f'已发 LANDING_TARGET {self.lt_sent}')

        if self.debug_period and target is not None \
                and self.frame_count % self.debug_period == 0:
            self._save_debug(frame, target)

    def _detect(self, frame, h_m, intr):
        """按 backend 出像素级候选：yolo 主通道 / hough 兜底 / both 融合。

        初始化后保证 backend 与 engine 状态自洽：yolo ⇒ engine 在；
        both ⇒ engine 在（否则已在启动时降为 hough）。
        """
        yolo, hough = [], []
        if self.backend in ('yolo', 'both'):
            results = self.model.predict(frame, imgsz=self.imgsz,
                                         conf=self.det_conf, verbose=False)
            r = results[0]
            if r.boxes is not None:
                for xywh, c in zip(r.boxes.xywh.tolist(),
                                   r.boxes.conf.tolist()):
                    bx, by, bw, bh = [float(v) for v in xywh]
                    yolo.append(vc.PixelDet(u=bx, v=by, a_px=bw, b_px=bh,
                                            conf=float(c), source='main'))
        if self.backend in ('hough', 'both'):
            hough = vc.hough_h_detect(frame, fx=intr.fx, h_m=h_m,
                                      params=self.hough_params)
        if self.backend == 'both':
            return vc.fuse_channels(yolo, hough)
        if self.backend == 'yolo':
            return yolo
        return hough

    def _publish_landing_target(self, x_flu, y_flu, h_m, diam, t_frame):
        """FLU → FRD（y、z 取反）后按 MAVLink LANDING_TARGET 语义填角度+位置。"""
        x, y, z = vc.flu_to_frd(x_flu, y_flu, -h_m)   # z_flu = −h_m（目标在下）
        now = time.time()
        if now - self.last_lt_send < self.min_interval:
            return
        self.last_lt_send = now
        m = LandingTarget()
        m.header.stamp = self._sec_to_stamp(t_frame)
        m.header.frame_id = self.get_name()
        m.target_num = 0
        m.frame = self.lt_frame
        dist = math.sqrt(x * x + y * y + z * z)
        if hasattr(m, 'angle_x'):
            m.angle_x = math.atan2(y, max(1e-3, abs(z)))   # 水平偏角（FRD 下 z 为正深度）
            m.angle_y = math.atan2(x, max(1e-3, abs(z)))   # 前后偏角
        m.distance = float(dist)
        if hasattr(m, 'size_x'):
            m.size_x = float(diam)
            m.size_y = float(diam)
        if hasattr(m, 'position'):
            m.position.x = float(x)
            m.position.y = float(y)
            m.position.z = float(z)
        if hasattr(m, 'type'):
            m.type = _MAV_LANDING_TARGET_TYPE_VISION_FIDUCIAL
        self.lt_pub.publish(m)
        self.lt_sent += 1

    def _publish_debug(self, x, y, z, diam, t_frame):
        p = PoseStamped()
        p.header.stamp = self._sec_to_stamp(t_frame)
        p.header.frame_id = 'cuadc_body_flu'
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.position.z = float(z)
        p.pose.orientation.x = float(diam)      # 复用桶契约语义：直径
        p.pose.orientation.z = 1.0              # 契约哨兵（调试消费方校验用）
        self.dbg_pub.publish(p)

    def _publish_heartbeat(self, t_frame):
        hb = Header()
        hb.stamp = self._sec_to_stamp(t_frame)
        hb.frame_id = self.get_name()
        self.hb_pub.publish(hb)

    @staticmethod
    def _sec_to_stamp(t_sec: float):
        from builtin_interfaces.msg import Time
        sec = int(t_sec)
        return Time(sec=sec, nanosec=int((t_sec - sec) * 1e9))

    def _save_debug(self, frame, det):
        vis = frame.copy()
        cv2.circle(vis, (int(det.u), int(det.v)), int(det.a_px / 2),
                   (0, 255, 0), 2)
        cv2.imwrite(f'/tmp/h_circle_{self.frame_count}.png', vis)


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = HCircleNode()
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
