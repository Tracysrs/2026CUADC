"""判读→数传填单文本纯逻辑（无 ROS 依赖，Windows 可单测）。

消费者 = recon_gcs_bridge_node（机载）：把 /cuadc/recon/classification 的窗口结论
格式化成短文本经 MAVLink STATUSTEXT 发地面（04 册 §2.4 填单主通道的 Jetson 侧源头）。

文本格式与 severity 口径：
  确认   侦察V13 #0 腐蚀品 c0.93        severity=NOTICE
  不足   侦察V13 #1 留空(不足)          severity=NOTICE（时间允许可人眼补判）
  混淆   侦察V13 #2 留空(混淆!)转人眼    severity=WARNING（MP 醒目，必须转人眼 FPV）
  越界   侦察V13 #3 类别11(越界!)        severity=NOTICE（类名表与判读不同源时兜底）

带宽口径：单行 UTF-8 ≤50 字节 = MAVLink1 STATUSTEXT 上限也安全（本机 MAVLink2@9600
上限 254，余量更大）；发布节奏 = 每 capture_request 一窗一发（hazard_recon_node
窗口收口才 publish），6 航点 ×3~5 标识 ≈ 20~30 行/架次，9600 空口无压力。

留空是合法答案（留空 0 分、错填 −100，宁空勿错）——文本必须带上留空原因，
让填单员一眼分清"可补判的不足"与"必须转人眼的混淆"。
"""

from types import SimpleNamespace

# 附件11 实表（2026-09-12 按扫描件逐页比对勘正，页序 = id 序；勿用标准 GHS 表，
# 勘误链见 06_使用说明书/11_错误与经验总结.md §2）。viewer / 桥 / 填单卡三方同源，
# 改训练集 data.yaml 必须同步这里。
CLASS_NAMES = [
    '爆炸品', '不燃气体', '刺激性', '放射性物品', '腐蚀品',
    '生物危害', '遇湿易燃物品', '有毒品', '自燃物品', '易燃',
]

# 与 mavros_msgs/StatusText 的 severity 常量一致（此处不能 import ROS，故镜像定义）
SEV_WARNING = 4
SEV_NOTICE = 5

# MAVLink1 STATUSTEXT text 字段上限（字节）；新格式入本表前先过 test 的字节断言
MAX_TEXT_BYTES = 50


def format_marker_line(viewpoint_seq, marker, class_names=None):
    """一条判读结论 → (STATUSTEXT 文本, severity)。

    marker 鸭子类型：需要 marker_index / class_id / confidence / ambiguous 属性
    （ReconMarker 直接可用；测试用 SimpleNamespace）。
    """
    if class_names is None:
        class_names = CLASS_NAMES
    idx = marker.marker_index
    head = f'侦察V{viewpoint_seq} #{idx}'
    if getattr(marker, 'ambiguous', False):
        return f'{head} 留空(混淆!)转人眼', SEV_WARNING
    cid = marker.class_id
    if cid < 0:
        return f'{head} 留空(不足)', SEV_NOTICE
    if cid >= len(class_names):
        return f'{head} 类别{cid}(越界!)', SEV_NOTICE
    return f'{head} {class_names[cid]} c{marker.confidence:.2f}', SEV_NOTICE


class ReconTextGate:
    """精确重复抑制：同窗同结论只发一次，改判/新窗自动重发。

    按 (viewpoint_seq, marker_index, class_id, ambiguous) 记账——confidence/frames
    的微小变化不触发重发（空口省带宽），类别结论变化才重发（改判必须让地面知道）。
    状态有界：超过 max_keys 丢一半旧账（防长会话无界增长，正常架次远用不到）。
    """

    def __init__(self, max_keys=512):
        self._sent = {}  # key -> True，dict 保插入序（裁剪时丢最旧的）
        self._max = max_keys

    def new_lines(self, viewpoint_seq, markers, class_names=None):
        """本条 ReconClassification 里没发过的结论 → [(text, severity), ...]。"""
        if class_names is None:
            class_names = CLASS_NAMES
        out = []
        for m in markers:
            key = (int(viewpoint_seq), int(m.marker_index),
                   int(m.class_id), bool(m.ambiguous))
            if key in self._sent:
                continue
            self._sent[key] = True
            out.append(format_marker_line(viewpoint_seq, m, class_names))
        if len(self._sent) > self._max:
            keep = list(self._sent)[len(self._sent) - self._max // 2:]
            self._sent = dict.fromkeys(keep, True)
        return out


def _self_check():
    """最小自检（不带测试框架也能跑：python -m cuadc_perception.recon_gcs_text）。"""
    m = SimpleNamespace(marker_index=0, class_id=4, confidence=0.93, ambiguous=False)
    text, sev = format_marker_line(13, m)
    assert text == '侦察V13 #0 腐蚀品 c0.93' and sev == SEV_NOTICE
    assert len(text.encode('utf-8')) <= MAX_TEXT_BYTES


if __name__ == '__main__':
    _self_check()
    print('recon_gcs_text self-check OK')
