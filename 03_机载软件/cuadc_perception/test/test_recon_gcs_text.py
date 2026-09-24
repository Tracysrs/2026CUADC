"""recon_gcs_text 单元测试（纯 stdlib，本机可直接跑，不需要 ROS）：

    cd 03_机载软件
    PYTHONPATH=cuadc_perception python -m unittest discover -s cuadc_perception/test -v

（PYTHONPATH 指向内层包目录——外层是 ament_python 工程根，直接 PYTHONPATH=.
会解析到外层同名目录而 import 失败。）

覆盖：文本格式与 severity（桥→MP 的填单口径）、50 字节上限（MAVLink1 也安全）、
ReconTextGate 去重语义（同窗同结论不发、改判/新窗重发）。
"""

import unittest
from types import SimpleNamespace

from cuadc_perception.recon_gcs_text import (
    CLASS_NAMES,
    MAX_TEXT_BYTES,
    SEV_NOTICE,
    SEV_WARNING,
    ReconTextGate,
    format_marker_line,
)


def marker(idx, class_id, conf=0.90, ambiguous=False):
    return SimpleNamespace(marker_index=idx, class_id=class_id,
                           confidence=conf, ambiguous=ambiguous)


class TestFormatMarkerLine(unittest.TestCase):

    def test_confirmed(self):
        text, sev = format_marker_line(13, marker(0, 4, 0.93))
        self.assertEqual(text, '侦察V13 #0 腐蚀品 c0.93')
        self.assertEqual(sev, SEV_NOTICE)

    def test_blank_insufficient(self):
        text, sev = format_marker_line(13, marker(1, -1))
        self.assertEqual(text, '侦察V13 #1 留空(不足)')
        self.assertEqual(sev, SEV_NOTICE)

    def test_ambiguous_goes_warning(self):
        text, sev = format_marker_line(13, marker(2, -1, ambiguous=True))
        self.assertEqual(text, '侦察V13 #2 留空(混淆!)转人眼')
        self.assertEqual(sev, SEV_WARNING)

    def test_class_id_out_of_range(self):
        text, _ = format_marker_line(13, marker(3, 11))
        self.assertEqual(text, '侦察V13 #3 类别11(越界!)')

    def test_custom_class_names(self):
        text, _ = format_marker_line(1, marker(0, 1, 0.80), ['甲', '乙'])
        self.assertEqual(text, '侦察V1 #0 乙 c0.80')

    def test_all_variants_within_mavlink1_bytes(self):
        """全部格式（含 3 位数 seq/idx、全 10 类、最大置信）UTF-8 ≤50 字节。"""
        variants = []
        for seq in (0, 7, 99, 999):
            for idx in (0, 9, 99):
                variants.append(format_marker_line(seq, marker(idx, -1)))
                variants.append(format_marker_line(seq, marker(idx, -1, ambiguous=True)))
                for cid in range(len(CLASS_NAMES)):
                    variants.append(format_marker_line(seq, marker(idx, cid, 1.0)))
        for text, _ in variants:
            self.assertLessEqual(len(text.encode('utf-8')), MAX_TEXT_BYTES, text)


class TestReconTextGate(unittest.TestCase):

    def setUp(self):
        self.gate = ReconTextGate()

    def test_first_window_all_sent(self):
        lines = self.gate.new_lines(13, [marker(0, 4), marker(1, -1)])
        self.assertEqual(len(lines), 2)

    def test_exact_repeat_suppressed(self):
        self.gate.new_lines(13, [marker(0, 4)])
        self.assertEqual(self.gate.new_lines(13, [marker(0, 4)]), [])

    def test_confidence_drift_not_resent(self):
        """置信漂移不重发（空口省带宽），类别结论变化才重发。"""
        self.gate.new_lines(13, [marker(0, 4, 0.90)])
        lines = self.gate.new_lines(13, [marker(0, 4, 0.95)])
        self.assertEqual(lines, [])

    def test_reclassification_resent(self):
        self.gate.new_lines(13, [marker(0, 4)])
        lines = self.gate.new_lines(14, [marker(0, 8)])
        self.assertEqual(lines, [('侦察V14 #0 自燃物品 c0.90', SEV_NOTICE)])

    def test_ambiguous_flip_resent(self):
        self.gate.new_lines(13, [marker(0, 4)])
        lines = self.gate.new_lines(14, [marker(0, 4, ambiguous=True)])
        self.assertEqual(lines, [('侦察V14 #0 留空(混淆!)转人眼', SEV_WARNING)])

    def test_new_viewpoint_without_change_resends(self):
        """新窗重复同结论也发：填单员按最新窗为准，跨窗重发是有意行为。"""
        self.gate.new_lines(13, [marker(0, 4)])
        lines = self.gate.new_lines(14, [marker(0, 4, 0.91)])
        self.assertEqual(len(lines), 1)

    def test_state_bounded(self):
        gate = ReconTextGate(max_keys=8)
        for seq in range(100):
            gate.new_lines(seq, [marker(0, seq % 10)])
        self.assertLessEqual(len(gate._sent), 8)


if __name__ == '__main__':
    unittest.main()
