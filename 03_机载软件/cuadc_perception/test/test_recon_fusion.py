"""recon_fusion 单元测试（纯 stdlib，本机可直接跑，不需要 ROS）：

    cd 03_机载软件
    PYTHONPATH=. python -m unittest discover -s cuadc_perception/test -v

覆盖《接口契约.md》§5 结论规则与 SSOT §4.4 拒识策略的可执行子集。
"""

import unittest

from cuadc_perception.recon_fusion import (
    CLASS_ID_BLANK,
    Detection,
    FrameGate,
    MarkerFusion,
    bbox_iou,
)


def det(u, v, class_id, conf, runner=0.0, w=40.0, h=40.0, t=0.0):
    return Detection(u=u, v=v, w=w, h=h, class_id=class_id,
                     confidence=conf, runner_up_conf=runner, stamp_s=t)


class TestBboxIou(unittest.TestCase):

    def test_same_box_is_one(self):
        self.assertAlmostEqual(bbox_iou(0, 0, 10, 10, 0, 0, 10, 10), 1.0)

    def test_disjoint_is_zero(self):
        self.assertEqual(bbox_iou(0, 0, 10, 10, 100, 100, 10, 10), 0.0)

    def test_half_overlap(self):
        # 10x10 与右移 5 的 10x10：交 5x10=50，并 150
        self.assertAlmostEqual(bbox_iou(0, 0, 10, 10, 5, 0, 10, 10), 50.0 / 150.0)


class TestFrameGate(unittest.TestCase):

    def setUp(self):
        self.gate = FrameGate()

    def test_ok(self):
        r = self.gate.check(det(0, 0, 9, 0.91, runner=0.40))
        self.assertTrue(r.accepted)
        self.assertFalse(r.need_boost)
        self.assertEqual(r.reason, 'ok')

    def test_low_confidence_rejected(self):
        r = self.gate.check(det(0, 0, 9, 0.5, runner=0.1))
        self.assertFalse(r.accepted)
        self.assertFalse(r.need_boost)
        self.assertEqual(r.reason, 'low_confidence')

    def test_low_margin_rejected(self):
        r = self.gate.check(det(0, 0, 9, 0.9, runner=0.7))  # margin=0.2 < 0.3
        self.assertFalse(r.accepted)
        self.assertEqual(r.reason, 'low_margin')

    def test_gray_confidence_needs_boost(self):
        r = self.gate.check(det(0, 0, 9, 0.75, runner=0.1))
        self.assertFalse(r.accepted)
        self.assertTrue(r.need_boost)
        self.assertIn('gray', r.reason)

    def test_gray_margin_needs_boost(self):
        r = self.gate.check(det(0, 0, 9, 0.9, runner=0.55))  # margin=0.35 ∈ [0.3,0.45)
        self.assertTrue(r.need_boost)

    def test_boosted_passes_gray_but_not_hard(self):
        self.assertTrue(self.gate.check_boosted(det(0, 0, 9, 0.75, runner=0.1)).accepted)
        self.assertFalse(self.gate.check_boosted(det(0, 0, 9, 0.5, runner=0.1)).accepted)

    def test_invalid_thresholds_rejected(self):
        with self.assertRaises(ValueError):
            FrameGate(reject_top1=0.9, gray_top1_high=0.8)


class TestMarkerFusion(unittest.TestCase):

    def make_fusion(self):
        return MarkerFusion()

    def test_confirmed_when_dominant(self):
        f = self.make_fusion()
        for i in range(6):
            f.update([det(100, 100, 9, 0.91, runner=0.4, t=i * 0.1)])
        v = f.verdicts()
        self.assertEqual(len(v), 1)
        self.assertEqual(v[0].class_id, 9)
        self.assertFalse(v[0].ambiguous)
        self.assertEqual(v[0].frames, 6)
        self.assertAlmostEqual(v[0].confidence, 0.91)

    def test_split_votes_is_ambiguous_blank(self):
        f = self.make_fusion()
        for i in range(5):
            f.update([det(100, 100, 8, 0.9, runner=0.1)])  # 自燃 5 票
        for i in range(4):
            f.update([det(100, 100, 6, 0.9, runner=0.1)])  # 遇湿易燃 4 票
        v = f.verdicts()[0]
        self.assertEqual(v.class_id, CLASS_ID_BLANK)
        self.assertTrue(v.ambiguous)  # 8 与 6 都达标 → 混淆拒识

    def test_qualified_but_not_dominant_is_ambiguous(self):
        f = self.make_fusion()
        for i in range(5):
            f.update([det(100, 100, 2, 0.9, runner=0.1)])  # 5 票达标
        for i in range(4):
            f.update([det(100, 100, 1, 0.5, runner=0.1)])  # 4 票不达标但非噪声
        v = f.verdicts()[0]
        self.assertEqual(v.class_id, CLASS_ID_BLANK)
        self.assertTrue(v.ambiguous)  # 5 < 3×4 → 无主导

    def test_insufficient_frames_is_blank_not_ambiguous(self):
        f = self.make_fusion()
        for i in range(3):
            f.update([det(100, 100, 7, 0.95, runner=0.1)])
        v = f.verdicts()[0]
        self.assertEqual(v.class_id, CLASS_ID_BLANK)
        self.assertFalse(v.ambiguous)
        self.assertTrue(v.insufficient)

    def test_median_conf_used_not_mean_extreme(self):
        f = self.make_fusion()
        confs = [0.82, 0.84, 0.86, 0.88, 0.30]  # 中位数 0.84（抗 0.30 离群），均值仅 0.74
        for i, c in enumerate(confs):
            f.update([det(100, 100, 4, c, runner=0.1)])
        v = f.verdicts()[0]
        self.assertEqual(v.class_id, 4)
        self.assertAlmostEqual(v.confidence, 0.84)

    def test_low_median_conf_blocks_confirm(self):
        f = self.make_fusion()
        for i in range(6):
            f.update([det(100, 100, 5, 0.75, runner=0.1)])  # 票够但中位 < 0.8
        v = f.verdicts()[0]
        self.assertEqual(v.class_id, CLASS_ID_BLANK)
        self.assertTrue(v.insufficient)

    def test_two_markers_associated_separately(self):
        f = self.make_fusion()
        for i in range(6):
            f.update([det(100, 100, 9, 0.9, runner=0.1),
                      det(600, 300, 2, 0.9, runner=0.1)])
        v = f.verdicts()
        self.assertEqual(len(v), 2)
        self.assertEqual({x.class_id for x in v}, {9, 2})
        self.assertEqual([x.marker_index for x in v], [0, 1])

    def test_reset_clears_state(self):
        f = self.make_fusion()
        f.update([det(100, 100, 9, 0.9, runner=0.1)])
        f.reset()
        self.assertEqual(f.verdicts(), [])


if __name__ == '__main__':
    unittest.main()
