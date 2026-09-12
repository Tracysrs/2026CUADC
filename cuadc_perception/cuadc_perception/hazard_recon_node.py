#!/usr/bin/env python3
"""真侦察判读节点（M4）：TensorRT engine 帧级检测 → recon_fusion 窗口融合 → 判读发布。

链路（接口契约.md §5）：
  相机 Image → YOLO engine 检测（坐标还原到全分辨率像素系）→ FrameGate 帧级门控
  （灰区 ROI 放大重推一次）→ MarkerFusion 窗口融合 → /cuadc/recon/classification。

握手语义复用 fake_recon_node（capture_done = "证据窗口已开启"，判读异步产出不阻塞
状态机）；发布语义与其一致：stamp = 取帧时刻（图像 header.stamp，相机驱动回调瞬间），
viewpoint_seq 对齐 capture_request，空结论也发。

检测模型上的 top1/margin 口径（契约 §5"帧级门控"在本节点的落地）：
  top1 = 该框置信度；runner-up = 同位置竞争框（IoU ≥ comp_iou）中**异类**最高置信度
  ——det 模型每框单类，类别竞争体现为同位置异类框的置信度对峙。

工程五件套落地项（SSOT §4.6）：
  - 模型 SHA-256 校验（expected_sha256 非空时强校验，不符拒启）；
  - 黑帧预热 3 帧（开机即热，journal 留痕，P0.3 systemd 自启的"预热自检"步）；
  - 每帧心跳（含空帧，fail-closed）；证据落盘 1s/张 JPEG + 结论 JSONL（原子写），
    磁盘剩余 <512MB 自动停证并告警。

systemd 自启（P0.3）：cuadc-perception.service 开机拉起本节点，加载+预热后待命。
"""

import hashlib
import json
import os
import shutil
import time
from datetime import datetime

import cv2
import numpy as np
import rclpy
from cuadc_interfaces.msg import ReconClassification, ReconMarker
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Header, String, UInt32
from ultralytics import YOLO

from cuadc_perception.recon_fusion import Detection, FrameGate, MarkerFusion


class HazardReconNode(Node):

    def __init__(self):
        super().__init__('hazard_recon')

        # ---- 话题 ----
        self.declare_parameter('image_topic', '/camera/image_raw')
        # 图像订阅 QoS：默认 RELIABLE——大图经 UDP-only 传输时 best_effort 丢一个
        # 分片 = 丢整帧（09-12 实测 10Hz 只吃到 ~2.5Hz）；RELIABLE 靠重传保整帧。
        self.declare_parameter('image_qos_reliable', True)
        self.declare_parameter('mission_state_topic', '/cuadc/mission_state')
        self.declare_parameter('request_topic', '/cuadc/recon/capture_request')
        self.declare_parameter('done_topic', '/cuadc/recon/capture_done')
        self.declare_parameter('classification_topic', '/cuadc/recon/classification')
        # 注意：/perception/heartbeat 是 M2 桶检测节点的契约心跳（契约 §2）；本节点是
        # 另一个生产者，默认发独立话题（契约 v1.2 新增行）。
        self.declare_parameter('heartbeat_topic', '/perception/heartbeat_recon')

        # ---- 模型与校验 ----
        self.declare_parameter('engine_path', '/home/nvidia/cuadc_models/best.engine')
        self.declare_parameter('expected_sha256', '')  # 非空 = 强校验，不符拒启
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('det_conf_thres', 0.10)  # 融合入口低阈，判真伪交给门控
        self.declare_parameter('nms_iou', 0.60)
        self.declare_parameter('comp_iou', 0.45)  # runner-up 竞争框的 IoU 门限

        # ---- 帧级门控 / 窗口融合（默认值 = recon_fusion.py = SSOT §4.4）----
        self.declare_parameter('reject_top1', 0.7)
        self.declare_parameter('reject_margin', 0.3)
        self.declare_parameter('gray_top1_high', 0.85)
        self.declare_parameter('gray_margin_high', 0.45)
        self.declare_parameter('boost_scale', 2.0)
        # 每帧 ROI 放大重推上限：灰区检测可能成打出现（拼图/多标识场景），
        # 不设限会把执行器拖到积压（09-12 实测 ack 定时器迟到 3s、窗口内只吃进 1 帧）
        self.declare_parameter('max_boost_per_frame', 4)
        self.declare_parameter('window_duration_s', 4.0)
        self.declare_parameter('assoc_iou', 0.3)
        self.declare_parameter('min_frames', 5)
        self.declare_parameter('min_median_conf', 0.8)
        self.declare_parameter('vote_ratio', 3.0)
        self.declare_parameter('require_recon_state', True)
        self.declare_parameter('ack_delay_s', 0.05)

        # ---- 证据落盘 ----
        self.declare_parameter('evidence_enabled', True)
        self.declare_parameter('evidence_dir', '~/recon_evidence')
        self.declare_parameter('debug_frame_log', False)  # 每帧打点：到达间隔+耗时

        gp = self.get_parameter
        self.engine_path = gp('engine_path').value
        self.expected_sha = gp('expected_sha256').value.strip().lower()
        self.imgsz = int(gp('imgsz').value)
        self.det_conf = float(gp('det_conf_thres').value)
        self.nms_iou = float(gp('nms_iou').value)
        self.comp_iou = float(gp('comp_iou').value)
        self.boost_scale = float(gp('boost_scale').value)
        self.max_boost = int(gp('max_boost_per_frame').value)
        self.window_duration = float(gp('window_duration_s').value)
        self.require_recon = bool(gp('require_recon_state').value)
        self.ack_delay = float(gp('ack_delay_s').value)
        self.evidence_wanted = bool(gp('evidence_enabled').value)
        self.evidence_root = os.path.expanduser(gp('evidence_dir').value)

        self._check_model_sha()

        # ---- 引擎加载 + 黑帧预热（P0.3 启动链，journal 留痕）----
        t0 = time.time()
        self.model = YOLO(self.engine_path, task='detect')
        self.class_names = self.model.names
        t_load = time.time() - t0

        black = np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8)
        t1 = time.time()
        for _ in range(3):
            self.model.predict(black, imgsz=self.imgsz, verbose=False)
        t_preheat = (time.time() - t1) * 1000
        self.get_logger().info(
            f'引擎就绪: {self.engine_path} (加载 {t_load:.2f}s, '
            f'黑帧预热 3 帧 {t_preheat:.0f}ms), {len(self.class_names)} 类')

        self.gate = FrameGate(
            reject_top1=float(gp('reject_top1').value),
            reject_margin=float(gp('reject_margin').value),
            gray_top1_high=float(gp('gray_top1_high').value),
            gray_margin_high=float(gp('gray_margin_high').value),
        )
        self.fusion_kwargs = dict(
            assoc_iou=float(gp('assoc_iou').value),
            min_frames=int(gp('min_frames').value),
            min_median_conf=float(gp('min_median_conf').value),
            vote_ratio=float(gp('vote_ratio').value),
        )

        # ---- 发布 / 订阅 ----
        qos_sensor = qos_profile_sensor_data
        self.cls_pub = self.create_publisher(
            ReconClassification, gp('classification_topic').value, 10)
        self.done_pub = self.create_publisher(UInt32, gp('done_topic').value, 10)
        self.hb_pub = self.create_publisher(
            Header, gp('heartbeat_topic').value, qos_sensor)
        self.bridge = CvBridge()
        if bool(gp('image_qos_reliable').value):
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            img_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        else:
            img_qos = qos_sensor
        self.create_subscription(
            Image, gp('image_topic').value, self.on_image, img_qos)
        self.create_subscription(
            PointStamped, gp('request_topic').value, self.on_request, 10)
        self.create_subscription(
            String, gp('mission_state_topic').value, self.on_state, 10)

        # ---- 窗口状态 ----
        self.current_state = ''
        self.acked_seqs = []
        self.fusion = None            # MarkerFusion，窗口开启时创建
        self.window_seq = -1
        self.window_deadline = 0.0
        self.window_last_stamp = None  # 窗口末帧取帧时刻
        self.window_dir = None
        self.last_evidence_t = 0.0
        self.last_frame = None         # 最近一帧原图（证据裁剪用）
        self.frame_count = 0
        self.debug_log = bool(gp('debug_frame_log').value)
        self._last_frame_t = 0.0

        self.create_timer(0.1, self.on_timer)  # 窗口超时看护
        self.get_logger().info(
            f'真判读就绪: image={gp("image_topic").value}, 窗口={self.window_duration:.1f}s, '
            f'门控 τ={self.gate.reject_top1}/m={self.gate.reject_margin}, '
            f'仅侦察态产出={self.require_recon}')

    # ---------------- 启动校验 ----------------

    def _check_model_sha(self):
        """模型 SHA-256 校验（SSOT §4.6 五件套之一），不符 = 拒启。"""
        if not self.expected_sha:
            self.get_logger().warn('未配置 expected_sha256，跳过模型校验（仅限调试）')
            return
        h = hashlib.sha256()
        with open(self.engine_path, 'rb') as f:
            for chunk in iter(lambda: f.read(1 << 20), b''):
                h.update(chunk)
        got = h.hexdigest()
        if got != self.expected_sha:
            raise RuntimeError(
                f'模型 SHA-256 不符！expected={self.expected_sha} got={got} '
                f'({self.engine_path})——疑似错拿错版本权重，拒绝启动')

    # ---------------- 窗口握手 ----------------

    def on_state(self, msg: String):
        self.current_state = msg.data

    def on_request(self, msg: PointStamped):
        seq = int(msg.point.x)
        if seq in self.acked_seqs:
            self.get_logger().warn(f'收到重复 capture_request seq={seq}，忽略')
            return
        self.acked_seqs.append(seq)

        # 证据目录与磁盘门禁（<512MB 停证，SSOT §5.3；盘容量查 home 所在分区）
        self.window_dir = None
        if self.evidence_wanted:
            free = shutil.disk_usage(os.path.expanduser('~')).free
            if free < 512 * 1024 * 1024:
                self.get_logger().warn(
                    f'磁盘剩余 {free >> 20}MB < 512MB，本次窗口停证据落盘（告警）')
            else:
                stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                self.window_dir = os.path.join(
                    self.evidence_root, f'viewpoint_{seq:03d}_{stamp}')
                os.makedirs(self.window_dir, exist_ok=True)

        self.fusion = MarkerFusion(**self.fusion_kwargs)
        self.window_seq = seq
        self.window_deadline = time.time() + self.window_duration
        self.window_last_stamp = None
        self.last_evidence_t = 0.0
        self.get_logger().info(
            f'capture_request seq={seq} → 证据窗口开启 ({self.window_duration:.1f}s)'
            + (f', 证据目录 {self.window_dir}' if self.window_dir else ''))

        # ack 尽快回：语义 = "证据窗口已开启"；判定异步产出，不阻塞状态机
        timer = self.create_timer(self.ack_delay, lambda: self._fire_ack(timer, seq))

    def _fire_ack(self, timer, seq):
        timer.cancel()
        done = UInt32()
        done.data = seq
        self.done_pub.publish(done)
        self.get_logger().info(f'capture_done seq={seq}（证据窗口已开启）')

    # ---------------- 主链路：图像 → 检测 → 门控 → 融合 ----------------

    def on_image(self, msg):
        try:
            t0 = time.time()
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._process_frame(msg, frame)
            if self.debug_log:
                now = time.time()
                self.get_logger().info(
                    f'帧打点: 到达间隔 {now - self._last_frame_t:.2f}s, '
                    f'处理 {(now - t0) * 1000:.0f}ms, 检测 {self._dbg_n} 框, '
                    f'detect {self._dbg_detect:.0f}ms, boost {self._dbg_boost:.0f}ms')
                self._last_frame_t = now
        except Exception as e:  # 单帧异常不丢节点：log + 继续心跳（fail-closed）
            self.get_logger().error(f'帧处理异常: {e}')

    def _process_frame(self, msg, frame):
        self.last_frame = frame
        # 契约 §3：stamp = 取帧时刻（相机驱动回调瞬间），不是本节点处理时刻；
        # 测试发布端不给 stamp（0）时才退化为 now。统一存 builtin_interfaces/Time。
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()

        t0 = time.time()
        dets = self._detect(frame)
        t1 = time.time()
        accepted = self._gate_with_boost(frame, dets)
        t2 = time.time()
        self._dbg_n, self._dbg_detect, self._dbg_boost = len(dets), (t1 - t0) * 1000, (t2 - t1) * 1000

        if self.fusion is not None:
            self.fusion.update(accepted)
            self.window_last_stamp = stamp
            self._save_evidence_jpeg(frame)

        # 心跳每帧发（含空帧）：停发 = 故障 = fail-closed（契约 §2）
        hb = Header()
        hb.stamp = stamp
        hb.frame_id = self.get_name()
        self.hb_pub.publish(hb)

        self.frame_count += 1
        if self.frame_count % 100 == 0:
            self.get_logger().debug(
                f'{self.frame_count} 帧, 本帧检测 {len(dets)}, 门控通过 {len(accepted)}')

    def _detect(self, frame):
        """engine 推理 → Detection 列表（全分辨率像素坐标 + runner-up 竞争度）。

        注意：runner-up 是 O(n²) 循环，必须先把张量一次性转成 Python float
        ——直接在循环里索引 torch 张量会因逐元素 GPU 算术拖到秒级
        （09-12 实测 52 框时 detect 2611ms，转 numpy 后 <5ms）。
        """
        r = self.model.predict(frame, imgsz=self.imgsz, conf=self.det_conf,
                               iou=self.nms_iou, verbose=False)[0]
        H, W = frame.shape[:2]
        boxes = r.boxes
        n = len(boxes)
        rows = [tuple(float(v) for v in row) for row in boxes.xywhn.cpu().numpy()]
        confs = [float(v) for v in boxes.conf.cpu().numpy()]
        clss = [int(v) for v in boxes.cls.cpu().numpy()]
        dets = []
        for i in range(n):
            cx, cy, w, h = rows[i]
            conf, cls = confs[i], clss[i]
            # runner-up = 同位置异类竞争框的最高置信度（det 模型的类别竞争口径）
            runner = 0.0
            for j in range(n):
                if j == i or clss[j] == cls:
                    continue
                if _iou_xywhn(rows[i], rows[j]) >= self.comp_iou:
                    runner = max(runner, confs[j])
            dets.append(Detection(u=cx * W, v=cy * H, w=w * W, h=h * H,
                                  class_id=cls, confidence=conf,
                                  runner_up_conf=runner,
                                  stamp_s=self.get_clock().now().nanoseconds / 1e9))
        return dets

    def _gate_with_boost(self, frame, dets):
        """帧级门控；灰区 ROI 全分辨率放大重推一次后复查（契约 §5）。

        灰区重推按置信度降序取前 max_boost_per_frame 个，防级联拖垮执行器。
        """
        accepted = []
        gray = []
        H, W = frame.shape[:2]
        for det in dets:
            g = self.gate.check(det)
            if g.accepted:
                accepted.append(det)
            elif g.need_boost:
                gray.append(det)
        gray.sort(key=lambda d: d.confidence, reverse=True)
        for det in gray[:self.max_boost]:
            boosted = self._boost(frame, det, W, H)
            if boosted is not None and self.gate.check_boosted(boosted).accepted:
                accepted.append(boosted)
        return accepted

    def _boost(self, frame, det, W, H):
        """裁剪 ROI（全分辨率）放大重推，坐标映射回原图像素系。"""
        hw = max(det.w, 32.0) * 0.75
        hh = max(det.h, 32.0) * 0.75
        x0, y0 = max(0, int(det.u - hw)), max(0, int(det.v - hh))
        x1, y1 = min(W, int(det.u + hw)), min(H, int(det.v + hh))
        crop = frame[y0:y1, x0:x1]
        if crop.size == 0:
            return None
        big = cv2.resize(crop, None, fx=self.boost_scale, fy=self.boost_scale,
                         interpolation=cv2.INTER_CUBIC)
        r = self.model.predict(big, imgsz=self.imgsz, conf=self.det_conf,
                               iou=self.nms_iou, verbose=False)[0]
        if len(r.boxes) == 0:
            return None
        i = int(r.boxes.conf.argmax())
        cx, cy, w, h = (float(v) for v in r.boxes.xywhn[i])
        BW, BH = big.shape[1], big.shape[0]
        return Detection(u=x0 + cx * BW / self.boost_scale,
                         v=y0 + cy * BH / self.boost_scale,
                         w=w * BW / self.boost_scale, h=h * BH / self.boost_scale,
                         class_id=int(r.boxes.cls[i]),
                         confidence=float(r.boxes.conf[i]),
                         runner_up_conf=0.0,  # 放大重推后无竞争框参照，复查只看硬门槛
                         stamp_s=det.stamp_s)

    # ---------------- 窗口收口 → 判读发布 + 证据 JSONL ----------------

    def on_timer(self):
        try:
            self._window_tick()
        except Exception as e:  # 窗口异常不丢节点：丢弃本窗口继续（fail-closed）
            self.get_logger().error(f'窗口收口异常: {e}')
            self.fusion = None

    def _window_tick(self):
        if self.fusion is None or time.time() < self.window_deadline:
            return
        verdicts = self.fusion.verdicts()
        seq = self.window_seq
        state = self.current_state or '?'
        if self.require_recon and 'RECON' not in self.current_state:
            self.get_logger().info(f'seq={seq} 判定丢弃：当前状态 {state} 非侦察态')
        else:
            msg = ReconClassification()
            # 窗口末帧取帧时刻（已是 builtin_interfaces/Time，无需转换）
            msg.header.stamp = (self.window_last_stamp
                                if self.window_last_stamp is not None
                                else self.get_clock().now().to_msg())
            msg.header.frame_id = self.get_name()
            msg.viewpoint_seq = seq
            lines = []
            for v in verdicts:
                m = ReconMarker()
                m.marker_index = v.marker_index
                m.class_id = v.class_id
                m.confidence = float(v.confidence)
                m.margin = float(v.margin)
                m.frames = int(v.frames)
                m.ambiguous = v.ambiguous
                m.evidence_path = (os.path.relpath(self.window_dir,
                                                   os.path.expanduser('~'))
                                   if self.window_dir else '')
                msg.markers.append(m)
                if self.window_dir:
                    crop = self._save_marker_crop(v)
                    lines.append(json.dumps({
                        'viewpoint_seq': seq,
                        'marker_index': v.marker_index,
                        'class_id': v.class_id,
                        'class_name': (self.class_names.get(v.class_id, '?')
                                       if v.class_id >= 0 else '留空'),
                        'confidence': float(v.confidence),
                        'margin': float(v.margin),
                        'frames': int(v.frames),
                        'ambiguous': v.ambiguous,
                        'insufficient': v.insufficient,
                        'crop': crop,
                        'stamp': datetime.now().isoformat(timespec='seconds'),
                    }, ensure_ascii=False))
            self.cls_pub.publish(msg)
            if lines:
                self._write_jsonl_atomic(lines)
            summary = ', '.join(
                f"#{v.marker_index}:"
                f"{'留空(混淆)' if v.ambiguous else '留空(不足)' if v.class_id < 0 else self.class_names.get(v.class_id, v.class_id)}"
                f" conf{v.confidence:.2f} {v.frames}帧" for v in verdicts) or '无结论'
            self.get_logger().info(f'判读结果 seq={seq} [{state}]: {summary}')
        self.fusion = None  # 窗口关闭

    # ---------------- 证据落盘 ----------------

    def _save_evidence_jpeg(self, frame):
        """窗口内 1s/张 JPEG（SSOT §4.4）。"""
        now = time.time()
        if now - self.last_evidence_t < 1.0:
            return
        self.last_evidence_t = now
        try:
            cv2.imwrite(os.path.join(self.window_dir, f'frame_{now:.0f}.jpg'), frame)
        except Exception as e:
            self.get_logger().warn(f'证据 JPEG 写入失败: {e}')

    def _save_marker_crop(self, v):
        """结论对应的标识裁剪图（最近一帧的框位置），返回窗口目录内文件名。"""
        acc = next((a for a in self.fusion.markers
                    if a.marker_index == v.marker_index), None)
        if acc is None or self.last_frame is None:
            return ''
        H, W = self.last_frame.shape[:2]
        x0 = max(0, int(acc.last_u - acc.last_w / 2))
        y0 = max(0, int(acc.last_v - acc.last_h / 2))
        x1 = min(W, int(acc.last_u + acc.last_w / 2))
        y1 = min(H, int(acc.last_v + acc.last_h / 2))
        if x1 <= x0 or y1 <= y0:
            return ''
        name = f'marker_{v.marker_index}.jpg'
        try:
            cv2.imwrite(os.path.join(self.window_dir, name),
                        self.last_frame[y0:y1, x0:x1])
            return name
        except Exception as e:
            self.get_logger().warn(f'证据裁剪写入失败: {e}')
            return ''

    def _write_jsonl_atomic(self, lines):
        """JSONL 原子写：tmp + os.replace，防半行（SSOT §4.4）。"""
        path = os.path.join(self.window_dir, 'result.jsonl')
        tmp = path + '.tmp'
        try:
            with open(tmp, 'a', encoding='utf-8') as f:
                f.write('\n'.join(lines) + '\n')
            os.replace(tmp, path)
        except Exception as e:
            self.get_logger().warn(f'证据 JSONL 写入失败: {e}')


def _iou_xywhn(a, b):
    ax1, ay1, ax2, ay2 = a[0]-a[2]/2, a[1]-a[3]/2, a[0]+a[2]/2, a[1]+a[3]/2
    bx1, by1, bx2, by2 = b[0]-b[2]/2, b[1]-b[3]/2, b[0]+b[2]/2, b[1]+b[3]/2
    iw = min(ax2, bx2) - max(ax1, bx1)
    ih = min(ay2, by2) - max(ay1, by1)
    if iw <= 0 or ih <= 0:
        return 0.0
    inter = iw * ih
    union = a[2]*a[3] + b[2]*b[3] - inter
    return inter / union if union > 0 else 0.0


def main(args=None):
    rclpy.init(args=args)
    node = HazardReconNode()
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
