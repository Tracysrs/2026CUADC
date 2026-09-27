"""recon_fusion 单元测试（纯 stdlib，本机可直接跑，不需要 ROS）：

    cd 03_机载软件
    PYTHONPATH=cuadc_perception python -m unittest discover -s cuadc_perception/test -v

（PYTHONPATH 指向包外层 cuadc_perception/ 目录——外层与内层同名，写 `.` 会被
namespace 包遮蔽报 ModuleNotFoundError，09-27 实测。）

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

    # ---- 移动判读候选开关（2026-09-27；默认关，准入凭 recon_eval 数据）----

    def test_best_k_conf_aggregation_confirms(self):
        # 8 帧置信爬升：全历史中位 (0.58+0.81)/2=0.695 <0.8 → 留空；
        # best_k（最优 5 帧 0.81~0.87 中位 0.83）→ 确认
        confs = [0.50, 0.52, 0.55, 0.58, 0.81, 0.83, 0.85, 0.87]

        def stream(f):
            for i, c in enumerate(confs):
                f.update([det(100, 100, 9, c, runner=0.1, t=i * 0.1)])

        f0 = MarkerFusion()
        stream(f0)
        v0 = f0.verdicts()[0]
        self.assertEqual(v0.class_id, CLASS_ID_BLANK)
        self.assertTrue(v0.insufficient)

        f1 = MarkerFusion(conf_agg='best_k')
        stream(f1)
        v1 = f1.verdicts()[0]
        self.assertEqual(v1.class_id, 9)
        self.assertFalse(v1.ambiguous)
        self.assertAlmostEqual(v1.confidence, 0.83)

    def test_invalid_conf_agg_rejected(self):
        with self.assertRaises(ValueError):
            MarkerFusion(conf_agg='top3')

    def test_assoc_predict_tracks_fast_mover(self):
        # 40px 框每帧右移 30px、dt=0.1s：纯 IoU(0.3) 关联断链碎票全留空；
        # 速度外推（含类级速度兜底冷启动）锁回单轨迹 → 确认
        def stream(f):
            for i in range(6):
                f.update([det(100 + 30 * i, 100, 9, 0.9, runner=0.1,
                              t=i * 0.1)])

        f0 = MarkerFusion()
        stream(f0)
        self.assertTrue(all(v.class_id == CLASS_ID_BLANK and v.insufficient
                            for v in f0.verdicts()))

        f1 = MarkerFusion(assoc_predict=True)
        stream(f1)
        confirmed = [v for v in f1.verdicts() if v.class_id == 9]
        self.assertEqual(len(confirmed), 1)
        self.assertGreaterEqual(confirmed[0].frames, 5)

    def test_assoc_predict_default_off_matches_legacy(self):
        # 默认参数下外推不生效：快速移动流仍碎票（与旧版行为一致）
        f = MarkerFusion()
        for i in range(6):
            f.update([det(100 + 30 * i, 100, 9, 0.9, runner=0.1, t=i * 0.1)])
        self.assertTrue(all(v.insufficient for v in f.verdicts()))


if __name__ == '__main__':
    unittest.main()
