#!/usr/bin/env python3
"""地面站查看器（M4 最简形态）：订阅 /cuadc/recon/classification，终端打印判读表。

只读节点，不发布任何话题、不连飞控。组员返航段按最新表填单。
显示约定（接口契约.md §5）：
  - 留空（class_id=-1）是合法答案：错填 -100、留空 0 分，宁空勿错；
  - ambiguous=true（混淆拒识）必须醒目提示 → 转人眼 FPV 备份判读；
  - class_names 参数默认与训练集 data.yaml / 附件11 对表，改训练集必须同步。
"""

import rclpy
from cuadc_interfaces.msg import ReconClassification
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

DEFAULT_CLASS_NAMES = [
    '爆炸品', '不燃气体', '刺激性', '放射性物品', '腐蚀品',
    '生物危害', '遇湿易燃物品', '有毒品', '自燃物品', '易燃',
]  # 附件11 实表（2026-09-12 按扫描件逐页比对勘正，页序 = id 序）

# 终端醒目色（数传终端不支持时退化为普通文本，不影响功能）
RED = '\033[91m'
GREEN = '\033[92m'
YELLOW = '\033[93m'
RESET = '\033[0m'


class ReconViewer(Node):

    def __init__(self):
        super().__init__('recon_viewer')
        self.declare_parameter('classification_topic', '/cuadc/recon/classification')
        self.declare_parameter('class_names', DEFAULT_CLASS_NAMES)
        self.class_names = list(self.get_parameter('class_names').value)
        qos = 10  # Reliable：判读结果不许丢
        self.create_subscription(
            ReconClassification, self.get_parameter('classification_topic').value,
            self.on_classification, qos)
        self.latest = {}  # marker_index -> (viewpoint_seq, ReconMarker)
        self.get_logger().info('判读查看器就绪（只读）。留空=合法答案；混淆=立即转人眼备份。')

    def on_classification(self, msg: ReconClassification):
        if not msg.markers:
            self.get_logger().info(f'viewpoint {msg.viewpoint_seq}: 本窗口无结论')
            return
        for m in msg.markers:
            self.latest[m.marker_index] = (msg.viewpoint_seq, m)

        print('\n===== 侦察判读表（组员填单依据，勿抄错行）=====')
        for idx in sorted(self.latest):
            seq, m = self.latest[idx]
            if m.ambiguous:
                verdict = f'{RED}留空(两类混淆!) → 转人眼FPV{RESET}'
            elif m.class_id < 0:
                verdict = f'{YELLOW}留空(证据不足) → 时间允许可人眼补判{RESET}'
            else:
                name = self.class_names[m.class_id] if m.class_id < len(self.class_names) \
                    else f'类别{m.class_id}(越界!)'
                verdict = f'{GREEN}{m.class_id} {name}{RESET}'
            print(f'  标识#{idx} (viewpoint {seq}): {verdict}  '
                  f'conf={m.confidence:.2f} margin={m.margin:.2f} frames={m.frames}')
        print('==========================================')

        lacking = [i for i, (_, m) in self.latest.items() if m.class_id < 0]
        if lacking:
            self.get_logger().warn(f'标识 {lacking} 暂为留空——填单前优先处理这几点')


def main(args=None):
    rclpy.init(args=args)
    node = ReconViewer()
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
