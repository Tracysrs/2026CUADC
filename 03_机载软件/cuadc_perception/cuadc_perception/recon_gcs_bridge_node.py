#!/usr/bin/env python3
"""判读→数传填单桥：/cuadc/recon/classification → MAVLink STATUSTEXT → 地面 MP。

链路（04 册 §2.4 填单主通道，Jetson 侧源头在本节点）：
  /cuadc/recon/classification（契约 §5，每 capture_request 一窗一发）
  → 短文本（recon_gcs_text.py 纯逻辑，单行 ≤50 字节）
  → /mavros/statustext/send（mavros sys_status 插件，mavros_msgs/StatusText）
  → FC（USB /dev/cuadc-fc）→ ArduPilot MAVLink_routing 跨口转发（STATUSTEXT 无
    目标字段 = 广播，转发到全部已学习路由；源码核实于 libraries/GCS_MAVLink/
    MAVLink_routing.cpp check_and_forward/forward）
  → TELEM2(@9600，09-24 定案) → 915MHz → 地面数传 → Mission Planner 消息栏 → 组员照文填单

边界与安全：
  - 只订阅判读话题、只发 statustext，不碰飞控任何状态（与 recon_viewer 同为只读消费者）；
  - STATUSTEXT 不受 SRx 流控（gcs_pump.py 头注同源结论），窗口级低频不挤占遥测；
  - 判读主链死亡 → 本桥无话可说（fail-silent），绝不编造/复述旧结论误导填单；
  - 填单规则不变：class_id 0..9=附件11 十类；-1=拒识留空（留空 0 分、错填 −100，
    宁空勿错）；ambiguous=true 必须留空并转人眼 FPV（severity=WARNING 醒目提示）。

用法（Jetson，前置 = mavros 已起、判读服务在跑）：
  ros2 launch cuadc_perception recon_gcs_bridge.launch.py
已并入 cuadc_start_perception.sh 随感知服务自启（09-26 拍板，真三跳验收后落
地；手工拉起仅调试用）。地面局域网内仍可跑 recon_viewer_node 看彩色判读表，
本桥是数传路径的等价物，两者可并存。
"""

import rclpy
from cuadc_interfaces.msg import ReconClassification
from mavros_msgs.msg import StatusText
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from cuadc_perception.recon_gcs_text import CLASS_NAMES, ReconTextGate, format_marker_line


class ReconGcsBridge(Node):

    def __init__(self):
        super().__init__('recon_gcs_bridge')
        self.declare_parameter('classification_topic', '/cuadc/recon/classification')
        self.declare_parameter('statustext_topic', '/mavros/statustext/send')
        self.declare_parameter('class_names', CLASS_NAMES)
        self.declare_parameter('dedupe', True)  # 同窗同结论只发一次；改判即重发

        gp = self.get_parameter
        self.class_names = list(gp('class_names').value)
        self.gate = ReconTextGate() if bool(gp('dedupe').value) else None

        qos = 10  # Reliable：判读结果一条不许丢（契约 §5 同款）
        self.statustext_pub = self.create_publisher(
            StatusText, gp('statustext_topic').value, qos)
        self.create_subscription(
            ReconClassification, gp('classification_topic').value,
            self.on_classification, qos)
        self.get_logger().info(
            f'填单桥就绪: {gp("classification_topic").value} → '
            f'{gp("statustext_topic").value}（前置: mavros 已起、TELEM2 数传在线）')

    def on_classification(self, msg: ReconClassification):
        if not msg.markers:
            self.get_logger().debug(f'viewpoint {msg.viewpoint_seq}: 本窗口无结论')
            return
        if self.gate is not None:
            lines = self.gate.new_lines(
                msg.viewpoint_seq, msg.markers, self.class_names)
        else:
            lines = [format_marker_line(msg.viewpoint_seq, m, self.class_names)
                     for m in msg.markers]
        for text, severity in lines:
            st = StatusText()
            st.severity = severity
            st.text = text
            self.statustext_pub.publish(st)
            self.get_logger().info(f'→ MP: {text}')


def main(args=None):
    rclpy.init(args=args)
    node = ReconGcsBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
