#!/usr/bin/env python3
"""白桶感知节点（真机，M2 核心）— 双通道检测 → 契约发布。

链路：/camera/image_raw/compressed（MJPG 1080p@30，best_effort）
      → odom 插值取"取帧时刻"的相对高度（P0.4 同源约定，见 时间同步设计.md）
      → 主通道 YOLOv8n-seg（TensorRT，可选）+ 副通道 LAB 分割（vision_core）
      → 通道融合 → 后处理四件（去重/独立性/直径先验/时序平滑）→ 机体系筒位
      → /perception/drop_buckets_body（PoseArray，契约 v1.3 §1）+ /perception/heartbeat（§2）

分级上线（设计文档 §5.2）：engine 文件缺失/SHA 不符/加载失败 → 自动降级 LAB-only
单通道（仍满足契约，置信打折），绝不因主通道缺失而停发——停发 = fail-closed。

仿真版（gz 直订）见 bucket_cv_perception_node.py；本节点与其共享 vision_core
算法语义，sim-to-real 零改契约。

与消费端的分工边界（防双重确认，设计文档 §4.3）：本节点只做逐帧中位数平滑，
**不确认**；连续帧确认/EMA/排名稳定/拉黑全部在 mission_node BucketMap。
"""

import os
import threading
import time
from collections import deque

import cv2
import numpy as np

from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header, String

from cuadc_perception import vision_core as vc

# datasheet 兜底内参（BL-500W-335：H81.8°/V66°，1080p 16:9 裁剪下 fx≈1108/fy≈831）
_HFOV_RAD = 1.4279
_VFOV_RAD = 1.1519


def _stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class BucketPerceptionNode(Node):

    def __init__(self):
        super().__init__('bucket_perception')
        gp = self.get_parameter

        # ---- 契约参数（接口契约.md v1.3 §1/§2）----
        self.declare_parameter('bucket_topic', '/perception/drop_buckets_body')
        self.declare_parameter('heartbeat_topic', '/perception/heartbeat')
        self.declare_parameter('mission_state_topic', '/cuadc/mission_state')
        self.declare_parameter('frame_id', 'cuadc_body_flu')
        self.declare_parameter('contract_version', 1.0)   # 契约哨兵
        self.declare_parameter('min_conf_out', 0.25)      # 消费端同款置信门

        # ---- 图像输入 ----
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('image_compressed', True)
        self.declare_parameter('image_qos_reliable', False)  # 与相机 best_effort 匹配
        self.declare_parameter('image_width', 1920)
        self.declare_parameter('image_height', 1080)

        # ---- 相机内参（工程五件套之三：正交性校验）----
        self.declare_parameter('calib_path', '/home/nvidia/cuadc_models/camera_calib.yaml')
        self.declare_parameter('allow_uncalibrated', True)
        self.declare_parameter('mount_rot_deg', 0.0)   # 装订旋转，悬停偏置目标实验标定

        # ---- 高度来源（单目解算，SSOT §4.2 修正版）----
        self.declare_parameter('odom_topic', '/mavros/local_position/odom')
        self.declare_parameter('plane_z_m', 0.30)      # 目标参考平面：桶口
        self.declare_parameter('ground_z_offset', 0.0) # EKF 原点在地面时为 0
        self.declare_parameter('odom_max_delay_s', 1.5)
        self.declare_parameter('odom_future_tol_s', 0.05)
        self.declare_parameter('odom_max_gap_s', 0.2)
        self.declare_parameter('odom_history_span_s', 3.0)

        # ---- 主通道 engine（可选；缺/坏自动 LAB-only）----
        self.declare_parameter('engine_path', '/home/nvidia/cuadc_models/bucket.engine')
        self.declare_parameter('expected_sha256', '')
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('seg_conf_thres', 0.25)

        # ---- 副通道 LAB（仿真节点 09-12 标定值）----
        for name, default in [('lab_l_min', 160.0), ('lab_l_offset', 10.0),
                              ('lab_ab_max_dev', 22.0), ('lab_min_area_px', 40.0),
                              ('lab_min_circularity', 0.60),
                              ('lab_diam_compensation', 1.15),
                              ('lab_conf_base', 0.55), ('lab_conf_circ_gain', 0.45)]:
            self.declare_parameter(name, default)

        # ---- 融合与后处理 ----
        for name, default in [('fuse_iou_thr', 0.5), ('fuse_w_main', 0.55),
                              ('fuse_w_aux', 0.45), ('fuse_agree_bonus', 0.10),
                              ('fuse_main_only_scale', 0.90),
                              ('fuse_aux_only_scale', 0.70)]:
            self.declare_parameter(name, default)
        self.declare_parameter('merge_dist_m', 0.02)
        self.declare_parameter('indep_min_dist_m', 0.20)
        self.declare_parameter('indep_min_diam_diff_m', 0.025)
        self.declare_parameter('smooth_window', 5)
        self.declare_parameter('smooth_assoc_gate_m', 0.25)

        # ---- 运行控制 ----
        self.declare_parameter('active_states', '')   # 空=全态处理；如 'SEARCH,ALIGN'
        self.declare_parameter('debug_save_period', 0)

        self.frame_id = gp('frame_id').value
        self.version = float(gp('contract_version').value)
        self.min_conf = float(gp('min_conf_out').value)
        self.img_w = int(gp('image_width').value)
        self.img_h = int(gp('image_height').value)
        self.plane_z = float(gp('plane_z_m').value)
        self.ground_z = float(gp('ground_z_offset').value)
        self.mount_rot = float(gp('mount_rot_deg').value)
        self.max_delay = float(gp('odom_max_delay_s').value)
        self.future_tol = float(gp('odom_future_tol_s').value)
        self.max_gap = float(gp('odom_max_gap_s').value)
        self.indep_dist = float(gp('indep_min_dist_m').value)
        self.indep_ddiff = float(gp('indep_min_diam_diff_m').value)
        self.merge_dist = float(gp('merge_dist_m').value)
        self.debug_period = int(gp('debug_save_period').value)
        state_str = str(gp('active_states').value).strip()
        self.active_states = [s.strip() for s in state_str.split(',') if s.strip()] \
            if state_str else []

        # ---- 内参（失败按 fail-closed 处理：比赛模式直接拒启）----
        calib_path = os.path.expanduser(str(gp('calib_path').value))
        self.intr, warns = vc.load_intrinsics(
            calib_path, self.img_w, self.img_h,
            hfov_rad=_HFOV_RAD, vfov_rad=_VFOV_RAD,
            allow_uncalibrated=bool(gp('allow_uncalibrated').value))
        for w in warns:
            self.get_logger().warn(w)
        self.get_logger().info(
            f'内参[{self.intr.source}]: fx={self.intr.fx:.1f} fy={self.intr.fy:.1f} '
            f'cx={self.intr.cx:.1f} cy={self.intr.cy:.1f}')

        # ---- 副通道/融合参数对象 ----
        self.lab_params = vc.LabParams(
            l_min=float(gp('lab_l_min').value), l_offset=float(gp('lab_l_offset').value),
            ab_max_dev=float(gp('lab_ab_max_dev').value),
            min_area_px=float(gp('lab_min_area_px').value),
            min_circularity=float(gp('lab_min_circularity').value),
            diam_compensation=float(gp('lab_diam_compensation').value),
            conf_base=float(gp('lab_conf_base').value),
            conf_circ_gain=float(gp('lab_conf_circ_gain').value))
        self.fuse_params = vc.FusionParams(
            iou_thr=float(gp('fuse_iou_thr').value), w_main=float(gp('fuse_w_main').value),
            w_aux=float(gp('fuse_w_aux').value), agree_bonus=float(gp('fuse_agree_bonus').value),
            main_only_scale=float(gp('fuse_main_only_scale').value),
            aux_only_scale=float(gp('fuse_aux_only_scale').value))
        self.smoother = vc.MedianSmoother(
            window=int(gp('smooth_window').value),
            assoc_gate_m=float(gp('smooth_assoc_gate_m').value))

        # ---- 主通道 engine（可选，分级上线）----
        self.model = self._load_engine_optional(
            os.path.expanduser(str(gp('engine_path').value)),
            str(gp('expected_sha256').value).strip(),
            int(gp('imgsz').value), float(gp('seg_conf_thres').value))

        # ---- odom 历史（P0.4：感知端也需要取帧时刻高度）----
        self.odom_buf = deque()          # (t_sec, z)
        self.odom_lock = threading.Lock()
        self.span_s = float(gp('odom_history_span_s').value)

        # ---- 发布 / 订阅 ----
        qos = qos_profile_sensor_data
        self.bucket_pub = self.create_publisher(
            PoseArray, gp('bucket_topic').value, qos)
        self.hb_pub = self.create_publisher(Header, gp('heartbeat_topic').value, qos)
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
        self.frame_count = 0
        self.detect_count = 0
        self.reject_diam_count = 0
        self.last_report = time.time()
        self.get_logger().info(
            f'白桶感知就绪: 通道={"seg+LAB" if self.model else "LAB-only(主通道未就绪)"}, '
            f'平面z={self.plane_z:.2f}m, min_conf={self.min_conf}, '
            f'active_states={self.active_states or "全态"}')

    # ---------------- 主通道 engine（可选加载） ----------------
    def _load_engine_optional(self, engine_path, expected_sha, imgsz, conf):
        """engine 缺失/SHA 不符/加载失败 → LAB-only 降级（分级上线，不拒启）。"""
        self.imgsz, self.seg_conf = imgsz, conf
        if not os.path.isfile(engine_path):
            self.get_logger().warn(
                f'主通道 engine 不存在({engine_path})，降级 LAB-only ——'
                f'白筒 seg 模型训练部署后自动升级双通道（设计文档 §5.2）')
            return None
        if expected_sha:
            got = vc.sha256_of_file(engine_path)
            if got != expected_sha:
                self.get_logger().error(
                    f'engine SHA-256 不符！expected={expected_sha} got={got}'
                    f'——拒用主通道，降级 LAB-only（防错拿错版本权重）')
                return None
        else:
            self.get_logger().warn('未配置 expected_sha256，跳过主通道校验（仅限调试）')
        try:
            from ultralytics import YOLO
            t0 = time.time()
            model = YOLO(engine_path, task='seg')
            black = np.zeros((imgsz, imgsz, 3), dtype=np.uint8)
            for _ in range(3):
                model.predict(black, imgsz=imgsz, verbose=False)
            self.get_logger().info(
                f'主通道 engine 就绪: {engine_path} (加载+预热 {time.time()-t0:.1f}s)')
            return model
        except Exception as e:  # noqa: BLE001 —— 加载失败不拖垮副通道
            self.get_logger().error(f'主通道加载失败({e})，降级 LAB-only')
            return None

    # ---------------- 订阅回调 ----------------
    def on_state(self, msg: String):
        self.current_state = msg.data

    def on_odom(self, msg: Odometry):
        t = _stamp_to_sec(msg.header.stamp)
        z = float(msg.pose.pose.position.z)
        with self.odom_lock:
            self.odom_buf.append((t, z))
            while self.odom_buf and t - self.odom_buf[0][0] > self.span_s:
                self.odom_buf.popleft()

    def _odom_z_at(self, t: float):
        """取帧时刻的高度插值（P0.4 同款容差：违反即 None，绝不外推）。"""
        with self.odom_lock:
            buf = list(self.odom_buf)
        if not buf:
            return None
        newest_t, newest_z = buf[-1]
        if t > newest_t + self.future_tol:
            return None                       # 未来戳
        if newest_t - t > self.max_delay:
            return None                       # 帧太旧（或时钟域不一致——查 stamp 来源）
        if t >= newest_t:
            return newest_z                   # 夹逼上界
        oldest_t, oldest_z = buf[0]
        if t <= oldest_t:
            return oldest_z                   # 夹逼下界（与 mission_node 一致）
        for i in range(len(buf) - 2, -1, -1):
            t0, z0 = buf[i]
            t1, z1 = buf[i + 1]
            if t0 <= t <= t1:
                if t1 - t0 > self.max_gap:
                    return None               # odom 断流段内不插值
                r = (t - t0) / max(1e-9, t1 - t0)
                return z0 + r * (z1 - z0)
        return None

    def on_image(self, msg):
        try:
            self._process(msg)
        except Exception as e:  # noqa: BLE001 —— 单帧异常不丢节点（fail-closed 心跳照发）
            self.get_logger().error(f'帧处理异常: {e}')
            self._publish([], _stamp_to_sec(msg.header.stamp))

    # ---------------- 主链路 ----------------
    def _process(self, msg):
        if self.image_compressed:
            frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8),
                                 cv2.IMREAD_COLOR)
            if frame is None:
                raise ValueError('JPEG 解码失败（截断帧？）')
        else:
            frame = np.frombuffer(msg.data, np.uint8).reshape(
                msg.height, msg.width, 3)

        # 契约 §3：stamp = 取帧时刻（相机驱动回调瞬间），非本节点处理时刻
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()
        t_frame = _stamp_to_sec(stamp)

        # 状态门控：非激活态只发空检测+心跳（保持契约频率与 fail-closed 语义）
        if self.active_states and self.current_state not in self.active_states:
            self._publish([], t_frame)
            return

        # 首帧分辨率校正（内参按参数分辨率加载）
        fh, fw = frame.shape[:2]
        intr = self.intr if (fw, fh) == (self.intr.width, self.intr.height) \
            else self.intr.scaled_to(fw, fh)

        oz = self._odom_z_at(t_frame)
        if oz is None:
            self.get_logger().warn(
                '取帧时刻无可用 odom 高度（太旧/断流/时钟域不一致），本帧发空检测',
                throttle_duration_sec=5.0)
            self._publish([], t_frame)
            return
        h_m = max(0.10, oz - self.ground_z - self.plane_z)

        # ---- 双通道 ----
        main_dets = self._detect_main(frame)
        aux_dets = vc.lab_white_detect(frame, self.lab_params)
        fused = vc.fuse_channels(main_dets, aux_dets, self.fuse_params)

        # ---- 解算 + 后处理四件 ----
        body = []
        for d in fused:
            x, y, z, diam = vc.ellipse_to_body(
                d.u, d.v, d.a_px, d.b_px, intr, h_m, self.plane_z, self.mount_rot)
            body.append(vc.BodyDet(x=x, y=y, z=z, diam_m=diam, conf=d.conf,
                                   source=d.source, u=d.u, v=d.v))
        body = vc.merge_same_frame(body, merge_dist_m=self.merge_dist)
        body = vc.enforce_independence(
            body, min_dist_m=self.indep_dist, min_diam_diff_m=self.indep_ddiff)
        body, rejected = vc.apply_diameter_prior(body)
        self.reject_diam_count += rejected
        body = self.smoother.update(body)
        body = [d for d in body if d.conf >= self.min_conf]
        body.sort(key=lambda d: -d.conf)
        body = body[:vc.MAX_POSES_PER_FRAME]

        self._publish(body, t_frame)

        self.frame_count += 1
        if body:
            self.detect_count += 1
        now = time.time()
        if now - self.last_report >= 10.0:
            rate = self.frame_count / max(1e-9, now - self.last_report)
            self.last_report = now
            self.get_logger().info(
                f'{rate:.1f}FPS, 帧 {self.frame_count} 检出帧 {self.detect_count}, '
                f'本帧 {len(body)} 筒, 高度 {h_m:.2f}m, 直径拒检累计 '
                f'{self.reject_diam_count}')
        if self.debug_period and body and self.frame_count % self.debug_period == 0:
            self._save_debug(frame, body)

    def _detect_main(self, frame):
        """YOLOv8n-seg：masks.xyn（原图归一化多边形）→ 全分辨率椭圆。"""
        if self.model is None:
            return []
        results = self.model.predict(frame, imgsz=self.imgsz,
                                     conf=self.seg_conf, verbose=False)
        r = results[0]
        out = []
        h, w = frame.shape[:2]
        if r.masks is not None and r.boxes is not None:
            polys = r.masks.xyn          # 每掩码一组 (x,y) 归一化多边形
            confs = [float(c) for c in r.boxes.conf.tolist()]
            for i, poly in enumerate(polys):
                if i >= len(confs) or len(poly) < 5:
                    continue
                cnt = (np.asarray(poly, dtype=np.float32)
                       * np.array([w, h], dtype=np.float32)).astype(np.int32)
                (eu, ev), (d_a, d_b), _ = cv2.fitEllipse(cnt)
                out.append(vc.PixelDet(u=float(eu), v=float(ev), a_px=float(d_a),
                                       b_px=float(d_b), conf=confs[i],
                                       source='main'))
        elif r.boxes is not None and len(r.boxes):
            # 掩码缺失兜底：用检测框（正下视圆的框即椭圆外接矩形）
            for xywh, c in zip(r.boxes.xywh.tolist(), r.boxes.conf.tolist()):
                bx, by, bw, bh = [float(v) for v in xywh]
                out.append(vc.PixelDet(u=bx, v=by, a_px=bw, b_px=bh,
                                       conf=float(c), source='main'))
        return out

    # ---------------- 发布（契约 v1.3 §1/§2） ----------------
    def _publish(self, body_dets, t_frame: float):
        msg = PoseArray()
        msg.header.stamp = self._sec_to_stamp(t_frame)
        msg.header.frame_id = self.frame_id
        for d in body_dets:
            pose = Pose()
            pose.position.x = float(d.x)
            pose.position.y = float(d.y)
            pose.position.z = float(d.z)
            pose.orientation.x = float(d.diam_m)   # 复用：筒口直径（m）
            pose.orientation.y = float(d.conf)     # 复用：置信度 [0,1]
            pose.orientation.z = self.version      # 复用：契约版本哨兵
            pose.orientation.w = 0.0               # 保留
            msg.poses.append(pose)
        self.bucket_pub.publish(msg)

        hb = Header()
        hb.stamp = self._sec_to_stamp(t_frame)
        hb.frame_id = self.get_name()
        self.hb_pub.publish(hb)

    @staticmethod
    def _sec_to_stamp(t_sec: float):
        from builtin_interfaces.msg import Time
        sec = int(t_sec)
        return Time(sec=sec, nanosec=int((t_sec - sec) * 1e9))

    def _save_debug(self, frame, body):
        vis = frame.copy()
        for d in body:
            cv2.circle(vis, (int(d.u), int(d.v)), 5, (0, 0, 255), 2)
            cv2.putText(vis, f'd={d.diam_m:.2f} c={d.conf:.2f} {d.source}',
                        (int(d.u) + 8, int(d.v)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.imwrite(f'/tmp/bucket_perception_{self.frame_count}.png', vis)


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = BucketPerceptionNode()
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
