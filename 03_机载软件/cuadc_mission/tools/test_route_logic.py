"""route_logic 单元测试（纯 stdlib）——test/test_route_logic.cpp 的 Python 镜像：

    cd 03_机载软件
    python -m unittest discover -s cuadc_mission/tools -v

三条防线与 C++ 版一一对应：罗盘→ENU 回归锚点（"横着搜"守门）、蛇形形态、
覆盖判据（带间无盲区）。改判据两边同步维护。
"""

import math
import unittest

from route_logic import (
    build_serpentine,
    field_to_local,
    heading_enu_rad_from_compass_deg,
    max_cross_track_distance,
    nadir_across_half_width,
)

PI = math.pi


def angle_diff(a: float, b: float) -> float:
    d = math.fmod(a - b, 2 * PI)
    if d > PI:
        d -= 2 * PI
    if d < -PI:
        d += 2 * PI
    return d


class TestHeadingConversion(unittest.TestCase):

    def test_compass_to_enu_anchors(self):
        # 罗盘 0/90/180/270（N/E/S/W）→ ENU π/2/0/−π/2/π；359° 越界 wrap
        cases = [(0.0, PI / 2), (90.0, 0.0), (180.0, -PI / 2),
                 (270.0, PI), (359.0, PI / 2 + math.radians(1.0))]
        for compass, expected in cases:
            self.assertAlmostEqual(
                angle_diff(heading_enu_rad_from_compass_deg(compass), expected),
                0.0, places=9)


class TestFieldToLocalAnchors(unittest.TestCase):

    def test_four_compass_anchors(self):
        # 机头方向 field(1,0) 必须落在地理正确的 ENU 方向上
        cases = [(0.0, 0.0, 1.0), (90.0, 1.0, 0.0),
                 (180.0, 0.0, -1.0), (270.0, -1.0, 0.0)]
        for compass, ex, ey in cases:
            enu = heading_enu_rad_from_compass_deg(compass)
            x, y, _ = field_to_local(enu, 0, 0, 0, 1, 0, 0)
            self.assertAlmostEqual(x, ex, places=9)
            self.assertAlmostEqual(y, ey, places=9)

    def test_left_and_translation(self):
        # 朝东时"左"= 北；朝北时"左"= 西；平移与 z 直通
        x, y, _ = field_to_local(0.0, 0, 0, 0, 0, 1, 0)
        self.assertAlmostEqual(x, 0.0, places=9)
        self.assertAlmostEqual(y, 1.0, places=9)
        x, y, _ = field_to_local(PI / 2, 0, 0, 0, 0, 1, 0)
        self.assertAlmostEqual(x, -1.0, places=9)
        self.assertAlmostEqual(y, 0.0, places=9)
        x, y, z = field_to_local(0.0, 10, 20, 1, 1, 2, 0.5)
        self.assertAlmostEqual(x, 11.0, places=9)
        self.assertAlmostEqual(y, 22.0, places=9)
        self.assertAlmostEqual(z, 1.5, places=9)

    def test_heading_wrong_90_guard(self):
        """守护断言：场地沿世界 +X 时出生朝北=bug 状态，航点必不在场地 +X。"""
        yaw_east = heading_enu_rad_from_compass_deg(90.0)
        x, y, _ = field_to_local(yaw_east, 0, 0, 0, 6, -2, 2)
        self.assertAlmostEqual(x, 6.0, places=9)
        self.assertAlmostEqual(y, -2.0, places=9)
        yaw_wrong = heading_enu_rad_from_compass_deg(0.0)
        bx, by, _ = field_to_local(yaw_wrong, 0, 0, 0, 6, -2, 2)
        self.assertGreater(abs(bx - 6.0), 1.0)


class TestSerpentine(unittest.TestCase):

    def test_along_x_six_lanes(self):
        # 搜索区现状：x 27.5~32.5（带长 5m），y ±4（带宽 8m），6 带
        r = build_serpentine(27.5, 32.5, -4.0, 4.0, 6, True)
        self.assertEqual(len(r), 12)
        self.assertAlmostEqual(r[0][0], 27.5, places=9)
        self.assertAlmostEqual(r[0][1], -4.0, places=9)
        self.assertAlmostEqual(r[1][0], 32.5, places=9)
        self.assertAlmostEqual(r[2][0], 32.5, places=9)   # 带 1 反向：x1→x0
        self.assertAlmostEqual(r[2][1], -2.4, places=9)
        self.assertAlmostEqual(r[3][0], 27.5, places=9)
        self.assertAlmostEqual(r[11][0], 27.5, places=9)  # 带 5（奇数）反向
        self.assertAlmostEqual(r[11][1], 4.0, places=9)
        for lane in range(6):                              # 带位均布 y=-4+8·lane/5
            self.assertAlmostEqual(r[2 * lane][1], -4.0 + 8.0 * lane / 5.0,
                                   places=9)
        one = build_serpentine(0, 10, -2, 2, 1, True)      # 单带居中
        self.assertEqual(len(one), 2)
        self.assertAlmostEqual(one[0][1], 0.0, places=9)

    def test_along_y_three_lanes(self):
        # 侦察区（09-12 重排）：带位沿 x 52.5~57.5 均布，带沿 y ±4 扫
        r = build_serpentine(52.5, 57.5, -4.0, 4.0, 3, False)
        self.assertEqual(len(r), 6)
        self.assertAlmostEqual(r[0][0], 52.5, places=9)
        self.assertAlmostEqual(r[0][1], -4.0, places=9)
        self.assertAlmostEqual(r[1][1], 4.0, places=9)
        self.assertAlmostEqual(r[2][0], 55.0, places=9)    # 带 1 反向：y1→y0
        self.assertAlmostEqual(r[2][1], 4.0, places=9)
        self.assertAlmostEqual(r[3][1], -4.0, places=9)
        self.assertAlmostEqual(r[4][0], 57.5, places=9)
        self.assertAlmostEqual(r[5][1], 4.0, places=9)


class TestCameraCoverage(unittest.TestCase):

    def test_half_width_and_no_blind_zone(self):
        across2 = nadir_across_half_width(2.0, 1.5, 848.0, 480.0)
        self.assertAlmostEqual(across2, 2.0 * math.tan(0.75) * 480.0 / 848.0,
                               places=9)                   # ≈1.0546
        self.assertTrue(0.80 < across2 < 1.20)             # 量级护栏
        across4 = nadir_across_half_width(4.0, 1.5, 848.0, 480.0)
        self.assertAlmostEqual(across4, 2.0 * across2, places=9)  # 线性于高度

        # 搜索区 6 带：网格采样 y，到最近带位横向距离 ≤ 半覆盖（无盲区）
        search = build_serpentine(27.5, 32.5, -4.0, 4.0, 6, True)
        lane_ys = [search[i][1] for i in range(0, len(search), 2)]
        worst = 0.0
        y = -4.0
        while y <= 4.0 + 1e-9:
            worst = max(worst, min(abs(y - ly) for ly in lane_ys))
            y += 0.1
        self.assertLessEqual(worst, across2 + 1e-9)        # 0.8 ≤ 1.05
        self.assertGreater(worst, 0.0)                     # 采样确实发生了
        self.assertAlmostEqual(max_cross_track_distance(8.0, 6), 0.8, places=9)

        # 侦察区 3 带（横向=x）：跨度 5m，最远 1.25m ≤ 4m 高半覆盖 2.11m
        recon = build_serpentine(52.5, 57.5, -4.0, 4.0, 3, False)
        lane_xs = [recon[i][0] for i in range(0, len(recon), 2)]
        worst_r = 0.0
        x = 52.5
        while x <= 57.5 + 1e-9:
            worst_r = max(worst_r, min(abs(x - lx) for lx in lane_xs))
            x += 0.1
        self.assertLessEqual(worst_r, across4 + 1e-9)
        self.assertAlmostEqual(max_cross_track_distance(5.0, 3), 1.25, places=9)
        self.assertAlmostEqual(max_cross_track_distance(8.0, 1), 4.0, places=9)


if __name__ == '__main__':
    unittest.main()
