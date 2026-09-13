"""drop_sim 闭环仿真测试（种子固定，完全确定）：

    cd 10_机载代码
    python -m unittest discover -s cuadc_mission/tools -v

断言口径说明：噪声模型是保守假设（视觉残差 σ2.5cm、标定残差 σ2cm、
风扰、舵机延迟 100ms），阈值给足余量；任何导致命中率和不变量
劣化的改动都应被视为回归。
"""

import unittest

from drop_logic import GateParams
from drop_sim import EpisodeResult, SimParams, run_episode, run_monte_carlo


class TestMonteCarloBaseline(unittest.TestCase):
    """保守场景蒙特卡洛：先大后小两瓶制（SSOT conservative）。"""

    def setUp(self):
        self.rs: list[EpisodeResult] = run_monte_carlo(12, base_seed=2026)

    def test_all_episodes_lock_three_buckets(self):
        self.assertTrue(all(r.locked for r in self.rs))

    def test_both_bottles_always_released(self):
        self.assertEqual(sum(r.n_fired for r in self.rs), 24)

    def test_hit_rate_and_cep(self):
        hits = sum(1 for r in self.rs for l in r.landings if l.hit)
        total = sum(len(r.landings) for r in self.rs)
        self.assertGreaterEqual(hits / total, 0.85)          # 23/24 = 96%
        landings25 = [l for r in self.rs for l in r.landings
                      if l.diameter_class == 2]
        self.assertGreaterEqual(len(landings25), 12)
        self.assertTrue(all(l.hit for l in landings25))      # 25cm 筒：全中
        cepts = sorted(r.cep for r in self.rs if r.landings)
        self.assertLess(cepts[len(cepts) // 2], 0.06)        # CEP 中位 < 6cm

    def test_no_invariant_violations(self):
        bad = [(r.seed if hasattr(r, 'seed') else '?', r.violations)
               for r in self.rs if r.violations]
        self.assertEqual(bad, [])

    def test_mission_completes(self):
        self.assertTrue(all(r.end_state == 'mission_complete' for r in self.rs))


class TestDropOrder(unittest.TestCase):

    def test_aggressive_small_bucket_first(self):
        r = run_episode(SimParams(seed=2026, drop_order='aggressive'))
        self.assertEqual(r.fired_classes, [0, 1])            # 15cm → 20cm
        self.assertEqual(r.violations, [])

    def test_conservative_large_bucket_first(self):
        r = run_episode(SimParams(seed=2026))
        self.assertEqual(r.fired_classes, [2, 1])            # 25cm → 20cm
        self.assertEqual(r.violations, [])


class TestVisionBlackout(unittest.TestCase):
    """I5 反盲投防线：释放/对准段断视觉 → 弃桶，绝不开火。"""

    def test_blackout_during_second_payload_abandons_not_blind_fires(self):
        r = run_episode(SimParams(seed=2026), blackout=(16.0, 20.0))
        # 第一瓶（25cm）在断视觉得以完成；第二瓶目标估计超龄 → 必须弃桶
        self.assertEqual(r.fired_classes, [2])
        self.assertEqual(r.abandoned_classes, [1])
        self.assertEqual(r.violations, [])
        self.assertEqual(r.end_state, 'mission_complete')

    def test_full_blackout_never_fires(self):
        """全程断视觉：锁不了也永远不该开火。"""
        r = run_episode(SimParams(seed=2026), blackout=(0.0, 200.0))
        self.assertEqual(r.n_fired, 0)
        self.assertEqual(r.violations, [])
        self.assertIn(r.end_state, ('search_no_lock', 'timeout'))


class TestStressDrops(unittest.TestCase):
    """压力：三瓶全投（覆盖第三载荷路径 + 拉黑后继续选目标）。"""

    def test_three_payloads_still_clean(self):
        rs = run_monte_carlo(8, base_seed=777, n_payloads=3)
        self.assertEqual(sum(r.n_fired for r in rs), 24)
        self.assertEqual(sum(len(r.violations) for r in rs), 0)
        # 三次投放必为三个不同筒（I3 的间接体现：命中记录来自不同 class 集合）
        for r in rs:
            self.assertLessEqual(max(r.fired_classes.count(c) for c in set(r.fired_classes)), 1)


if __name__ == '__main__':
    unittest.main()
