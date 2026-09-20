"""odom_interp 单元测试（纯 stdlib）——test/test_odom_interp.cpp 的 Python 镜像：

    cd 03_机载软件
    python -m unittest discover -s cuadc_mission/tools -v

覆盖 Re0 十二项回归清单的时间域三项：②时间回跳（超老夹逼/拒绝）、③插值
等号边界（端点/中间样本恰等）、④大间隔（严格大于才拒，恰等放行）。
"""

import math
import unittest
from collections import deque

from odom_interp import InterpParams, Sample, interpolate

P = InterpParams()   # 0.05 / 1.5 / 0.2


def hist(*pts):
    return deque(Sample(*p) for p in pts)


class TestInterpolate(unittest.TestCase):

    def test_empty_history_rejected(self):
        self.assertEqual(interpolate(deque(), 1.0, P).status, 'empty')

    def test_future_tol_boundary_returns_newest(self):
        """恰等未来戳容差 → 放行，返回 newest（容差取 0.0625=2^-4 保证
        FP 减法精确等号；0.05 类十进制值在二进制下会差出一个 ULP）。"""
        p2 = InterpParams(future_tol_s=0.0625)
        h = hist((0.0, 0, 0, 0, 0), (1.0, 1, 0, 0, 0))
        r = interpolate(h, 1.0625, p2)
        self.assertEqual(r.status, 'ok')
        self.assertAlmostEqual(r.out.t, 1.0)
        self.assertAlmostEqual(r.out.x, 1.0)
        self.assertEqual(interpolate(h, 1.0626, p2).status, 'future_stamp')

    def test_future_stamp_rejected(self):
        h = hist((0.0, 0, 0, 0, 0), (1.0, 1, 0, 0, 0))
        self.assertEqual(interpolate(h, 1.051, P).status, 'future_stamp')

    def test_too_old_boundary_clamps_to_oldest(self):
        """恰等 max_delay（-1.5）→ 夹逼返回 oldest（超老但在延迟窗内不丢弃）。"""
        h = hist((0.0, 5, 5, 2, 0.3), (1.0, 6, 5, 2, 0.3))
        r = interpolate(h, -0.5, P)
        self.assertEqual(r.status, 'ok')
        self.assertAlmostEqual(r.out.x, 5.0)
        self.assertAlmostEqual(r.out.yaw, 0.3)

    def test_too_old_rejected(self):
        h = hist((0.0, 5, 5, 2, 0.3), (1.0, 6, 5, 2, 0.3))
        self.assertEqual(interpolate(h, -0.51, P).status, 'too_old')

    def test_exact_middle_sample_returned_verbatim(self):
        """t 恰等于中间样本戳 → 原样返回该样本（r=1 走插值分支值恒等；
        样本间隔 0.1s 须 ≤ max_gap，间隔超限被拒属正确行为）。"""
        h = hist((0.0, 0, 0, 0, 0), (0.1, 1, 1, 1, 0.1), (0.2, 2, 2, 2, 0.2))
        r = interpolate(h, 0.1, P)
        self.assertEqual(r.status, 'ok')
        self.assertAlmostEqual(r.out.t, 0.1)
        self.assertAlmostEqual(r.out.x, 1.0)
        self.assertAlmostEqual(r.out.y, 1.0)
        self.assertAlmostEqual(r.out.yaw, 0.1)

    def test_midpoint_lerp(self):
        h = hist((0.0, 0, 0, 2, 0.0), (0.2, 1, 2, 2.2, 0.5))
        r = interpolate(h, 0.1, P)
        self.assertEqual(r.status, 'ok')
        self.assertAlmostEqual(r.out.x, 0.5)
        self.assertAlmostEqual(r.out.y, 1.0)
        self.assertAlmostEqual(r.out.z, 2.1)
        self.assertAlmostEqual(r.out.yaw, 0.25)

    def test_yaw_wrap_shortest_path(self):
        """a=+3.1, b=-3.1（跨 ±π）：最短角差 +0.0832，中点 ≈ π。"""
        h = hist((0.0, 0, 0, 0, 3.1), (0.2, 0, 0, 0, -3.1))
        r = interpolate(h, 0.1, P)
        self.assertEqual(r.status, 'ok')
        self.assertAlmostEqual(r.out.yaw, math.pi, places=6)

    def test_max_gap_boundary(self):
        """恰 0.2s 间隔放行（严格大于才拒）；0.21s 拒绝 gap_break。"""
        h = hist((0.0, 0, 0, 0, 0), (0.2, 1, 0, 0, 0))
        r = interpolate(h, 0.1, P)
        self.assertEqual(r.status, 'ok')
        self.assertAlmostEqual(r.out.x, 0.5)
        h2 = hist((0.0, 0, 0, 0, 0), (0.21, 1, 0, 0, 0))
        self.assertEqual(interpolate(h2, 0.1, P).status, 'gap_break')

    def test_time_jump_back_clamps_to_oldest(self):
        """时间回跳（t 早于 oldest 但在 max_delay 窗内）→ 夹逼 oldest。"""
        h = hist((0.0, 1, 0, 0, 0), (0.1, 1.1, 0, 0, 0))
        r = interpolate(h, -0.02, P)
        self.assertEqual(r.status, 'ok')
        self.assertAlmostEqual(r.out.x, 1.0)


if __name__ == '__main__':
    unittest.main()
