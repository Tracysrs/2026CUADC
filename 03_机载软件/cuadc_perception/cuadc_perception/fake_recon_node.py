#!/usr/bin/env python3
"""假判读节点：按《接口契约.md》§5 发布模拟侦察判读 + 演示握手语义。

用途（M4 前不需要真模型就能联调）：
  1. 验证状态机 capture_request/done 握手（序号去重、1s 超时兜底不被阻塞）；
  2. 作为 /cuadc/recon/classification 的标准发布端，联调地面站查看器；
  3. SITL 演示：假感知 + 假判读 + 查看器即可走通全链路显示。

关键语义（真判读节点 M4 在本包实现时必须复用）：
  capture_done = "该视点证据窗口已开启"，绝不等价于"判定完成"——
  状态机侧 1s 超时兜底只等 ack；判读异步产出，不回阻塞任务。
  classification.header.stamp = 融合窗口末帧的取帧时刻（这里 = now - 模拟延迟）。

场景参数 scenario：
  normal    3 个标识全部高置信确认（默认）
  ambiguous 某标识两类票数竞争 → 拒识留空（ambiguous=true）
  blank     全部证据不足 → 留空（检验"留空是合法答案"的显示链路）
"""

import random

import rclpy
from cuadc_interfaces.msg import ReconClassification, ReconMarker
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String, UInt32


class FakeReconNode(Node):

    def __init__(self):
        super().__init__('fake_recon')

        # ---- 话题 ----
        self.declare_parameter('mission_state_topic', '/cuadc/mission_state')
        self.declare_parameter('request_topic', '/cuadc/recon/capture_request')
        self.declare_parameter('done_topic', '/cuadc/recon/capture_done')
        self.declare_parameter('classification_topic', '/cuadc/recon/classification')

        # ---- 时序（联调超时行为用）----
        self.declare_parameter('ack_delay_s', 0.1)      # capture_done 延迟（>1.0 可测超时兜底）
        self.declare_parameter('verdict_delay_s', 2.0)  # 融合窗口时长（判定异步性演示）
        self.declare_parameter('require_recon_state', True)  # 仅 RECON* 态产出判定

        # ---- 场景 ----
        self.declare_parameter('scenario', 'normal')    # normal / ambiguous / blank
        self.declare_parameter('class_ids', [9, 2, 0])  # 易燃 / 刺激性 / 爆炸品（附件11 表，09-12 勘正）
        self.declare_parameter('confidences', [0.91, 0.88, 0.85])
        self.declare_parameter('margins', [0.44, 0.50, 0.60])
        self.declare_parameter('frames', [12, 9, 7])

        gp = self.get_parameter
        self.ack_delay = max(0.0, gp('ack_delay_s').value)
        self.verdict_delay = max(0.0, gp('verdict_delay_s').value)
        self.require_recon = bool(gp('require_recon_state').value)
        self.scenario = gp('scenario').value
        self.class_ids = [int(v) for v in gp('class_ids').value]
        self.confs = list(gp('confidences').value)
        self.margins = list(gp('margins').value)
        self.frames = [int(v) for v in gp('frames').value]
        if not (len(self.class_ids) == len(self.confs) == len(self.margins) == len(self.frames)):
            raise RuntimeError('class_ids/confidences/margins/frames 参数长度必须一致')
        if self.scenario not in ('normal', 'ambiguous', 'blank'):
            raise RuntimeError("scenario 只支持 normal / ambiguous / blank")

        qos = 10  # 判读是低频结构化结果，Reliable 不许丢（与检测流 best_effort 相反）
        self.state_sub = self.create_subscription(
            String, gp('mission_state_topic').value, self.on_state, qos)
        self.req_sub = self.create_subscription(
            PointStamped, gp('request_topic').value, self.on_request, qos)
        self.done_pub = self.create_publisher(UInt32, gp('done_topic').value, qos)
        self.cls_pub = self.create_publisher(
            ReconClassification, gp('classification_topic').value, qos)

        self.current_state = ''
        self.acked_seqs = []
        self.get_logger().info(
            f'假判读启动: scenario={self.scenario}, {len(self.class_ids)} 个标识, '
            f'ack={self.ack_delay:.2f}s, 判定窗口={self.verdict_delay:.1f}s, '
            f'仅侦察态产出={self.require_recon}')

    def on_state(self, msg: String):
        self.current_state = msg.data

    def on_request(self, msg: PointStamped):
        seq = int(msg.point.x)
        if seq in self.acked_seqs:
            self.get_logger().warn(f'收到重复 capture_request seq={seq}，忽略（对方应去重 ack）')
            return
        self.acked_seqs.append(seq)

        # ack 尽快回：语义 = 证据窗口已开启（1s 超时兜底只等这个）；
        # 判定异步产出：不阻塞任务，数传送达地面站供返航段读。
        # rclpy 定时器是周期性的，触发后自取消 = 单次。
        # 闭包迟到绑定：触发在赋值完成后，函数变量届时已指向定时器本体。
        ack_timer = self.create_timer(
            self.ack_delay,
            lambda: self._fire_once(ack_timer, lambda: self.send_done(seq)))
        verdict_timer = self.create_timer(
            self.ack_delay + self.verdict_delay,
            lambda: self._fire_once(verdict_timer, lambda: self.publish_verdict(seq)))

    @staticmethod
    def _fire_once(timer, fn):
        timer.cancel()
        fn()

    def send_done(self, seq: int):
        done = UInt32()
        done.data = seq
        self.done_pub.publish(done)
        self.get_logger().info(f'capture_done seq={seq}（证据窗口已开启）')

    def publish_verdict(self, seq: int):
        if self.require_recon and 'RECON' not in self.current_state:
            self.get_logger().info(
                f'seq={seq} 判定丢弃：当前状态 {self.current_state or "?"} 非侦察态')
            return

        msg = ReconClassification()
        msg.header.stamp = (self.get_clock().now() - Duration(seconds=self.verdict_delay)).to_msg()
        msg.header.frame_id = 'fake_recon'
        msg.viewpoint_seq = seq

        for i, (cid, conf, margin, n) in enumerate(
                zip(self.class_ids, self.confs, self.margins, self.frames)):
            m = ReconMarker()
            m.marker_index = i
            if self.scenario == 'blank':
                m.class_id = -1
                m.ambiguous = False
                m.confidence = conf
                m.margin = margin
                m.frames = max(1, n // 3)  # 证据不足
            elif self.scenario == 'ambiguous' and i == 0:
                m.class_id = -1            # 火焰三件套混淆 → 拒识留空
                m.ambiguous = True
                m.confidence = conf
                m.margin = 0.05            # top1≈top2
                m.frames = n
            else:
                m.class_id = cid
                m.ambiguous = False
                m.confidence = conf + random.uniform(-0.01, 0.01)
                m.margin = margin
                m.frames = n
            m.evidence_path = f'recon_evidence/demo/viewpoint_{seq}/marker_{i}'
            msg.markers.append(m)

        self.cls_pub.publish(msg)
        summary = ', '.join(
            f'#{m.marker_index}:{"留空(混淆)" if m.ambiguous else "留空(不足)" if m.class_id < 0 else m.class_id}'
            f' conf{m.confidence:.2f} {m.frames}帧'
            for m in msg.markers)
        self.get_logger().info(f'判读结果 seq={seq} [{self.current_state or "?"}]: {summary}')


def main(args=None):
    rclpy.init(args=args)
    node = FakeReconNode()
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
