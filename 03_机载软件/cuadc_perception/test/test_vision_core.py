#!/usr/bin/env python3
"""vision_core 单元测试（OpenCV + numpy，本机可直接跑，不需要 ROS）：

    cd 03_机载软件
    PYTHONPATH=. python -m unittest discover -s cuadc_perception/test -v

覆盖《视觉算法设计》§4 的可执行子集：内参/单目解算/LAB 分割/Hough H 圆/
后处理四件/双通道融合/时序平滑/FLU→FRD。合成帧全部程序化渲染，真值已知。
"""

import hashlib
import math
import os
import tempfile
import unittest

import cv2
import numpy as np

from cuadc_perception import vision_core as vc


def make_ground(w, h, bgr=(180, 90, 40), noise_sigma=3.0, seed=7):
    """蓝地底帧（有效区地面颜色为蓝色，细则 3.1.2）。"""
    rng = np.random.default_rng(seed)
    frame = np.full((h, w, 3), bgr, dtype=np.float32)
    if noise_sigma > 0:
        frame += rng.normal(0.0, noise_sigma, frame.shape)
    return np.clip(frame, 0, 255).astype(np.uint8)


def draw_bucket(frame, cx, cy, diam_px):
    """白色圆筒正下视投影 = 白色圆（硬边合成，无侧壁）。"""
    r = diam_px / 2.0
    cv2.circle(frame, (int(round(cx)), int(round(cy))), int(round(r)),
               (255, 255, 255), -1, lineType=cv2.LINE_8)
    return frame


def draw_h_circle(frame, cx, cy, r_px, bar_frac=0.13, bar_half=0.55,
                  cross=0.10, bar_len=0.62):
    """H 圆制式（设计文档 §4.6 采集卡同款）：白圆底 + 黑 H（两竖杠 + 横杠）。"""
    cv2.circle(frame, (int(cx), int(cy)), int(r_px), (255, 255, 255), -1)
    color = (0, 0, 0)
    bw = max(2, int(r_px * bar_frac))
    for sx in (-1, 1):
        x = int(cx + sx * bar_half * r_px)
        cv2.rectangle(frame, (x - bw // 2, int(cy - bar_len * r_px)),
                      (x + bw // 2, int(cy + bar_len * r_px)), color, -1)
    cv2.rectangle(frame, (int(cx - bar_half * r_px), int(cy - cross * r_px)),
                  (int(cx + bar_half * r_px), int(cy + cross * r_px)), color, -1)
    return frame


FX = 1000.0


class TestIntrinsics(unittest.TestCase):

    def test_from_fov_169_crop(self):
        intr = vc.Intrinsics.from_fov(1920, 1080, math.radians(81.8),
                                      math.radians(66.0))
        self.assertAlmostEqual(intr.fx, 1108, delta=3)
        self.assertAlmostEqual(intr.fy, 831, delta=3)
        self.assertEqual(intr.source, 'datasheet')

    def test_from_fov_square_pixel_fallback(self):
        intr = vc.Intrinsics.from_fov(1000, 1000, math.radians(90.0))
        self.assertAlmostEqual(intr.fx, intr.fy)
        self.assertAlmostEqual(intr.fx, 500.0 / math.tan(math.radians(45.0)))

    def test_validate_ok_and_fail(self):
        ok = vc.Intrinsics(fx=1000, fy=900, cx=960, cy=540,
                           width=1920, height=1080, source='calib')
        ok.validate()
        bad_cx = vc.Intrinsics(fx=1000, fy=900, cx=10, cy=540,
                               width=1920, height=1080, source='calib')
        with self.assertRaises(vc.CalibrationError):
            bad_cx.validate()
        bad_ratio = vc.Intrinsics(fx=500, fy=1800, cx=960, cy=540,
                                  width=1920, height=1080, source='calib')
        with self.assertRaises(vc.CalibrationError):
            bad_ratio.validate()

    def test_load_from_yaml(self):
        intr_expected = dict(fx=1112.0, fy=831.0, cx=960.0, cy=540.0,
                             dist_coeffs=[-0.1, 0.05, 0.0, 0.0, 0.0],
                             source='calib', reproj_error_px=0.3,
                             image_width=1920, image_height=1080)
        import yaml
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, 'calib.yaml')
            with open(path, 'w', encoding='utf-8') as f:
                yaml.safe_dump(intr_expected, f)
            intr, warns = vc.load_intrinsics(path, 1920, 1080)
            self.assertEqual(intr.source, 'calib')
            self.assertAlmostEqual(intr.fx, 1112.0)
            self.assertEqual(len(warns), 0)
            # 分辨率不一致 → 等比缩放并告警
            intr2, warns2 = vc.load_intrinsics(path, 960, 540)
            self.assertAlmostEqual(intr2.fx, 556.0, delta=1.0)
            self.assertEqual(len(warns2), 1)

    def test_load_missing_fallback_and_strict(self):
        intr, warns = vc.load_intrinsics('/nonexistent/calib.yaml', 1920, 1080)
        self.assertEqual(intr.source, 'datasheet')
        self.assertGreaterEqual(len(warns), 1)
        with self.assertRaises(vc.CalibrationError):
            vc.load_intrinsics('/nonexistent/calib.yaml', 1920, 1080,
                               allow_uncalibrated=False)

    def test_scaled_to(self):
        intr = vc.Intrinsics(fx=1112, fy=831, cx=960, cy=540,
                             width=1920, height=1080)
        s = intr.scaled_to(960, 540)
        self.assertAlmostEqual(s.fx, 556.0)
        self.assertAlmostEqual(s.cx, 480.0)

    def test_sha256_of_file(self):
        with tempfile.NamedTemporaryFile(delete=False) as f:
            f.write(b'cuadc-vision')
            path = f.name
        try:
            self.assertEqual(vc.sha256_of_file(path),
                             hashlib.sha256(b'cuadc-vision').hexdigest())
        finally:
            os.unlink(path)


class TestEllipseToBody(unittest.TestCase):

    def make_intr(self, fx=FX, fy=FX):
        return vc.Intrinsics(fx=fx, fy=fy, cx=960, cy=540,
                             width=1920, height=1080)

    def test_center_and_diameter(self):
        intr = self.make_intr()
        x, y, z, d = vc.ellipse_to_body(960, 540, 100, 100, intr, h_m=2.0,
                                        plane_z_m=0.30)
        self.assertAlmostEqual(x, 0.0, places=9)
        self.assertAlmostEqual(y, 0.0, places=9)
        self.assertAlmostEqual(z, -2.0, places=9)
        self.assertAlmostEqual(d, 0.20, places=6)

    def test_offset_direction(self):
        intr = self.make_intr()
        # 图像右方 (du>0) → 机体 -y（FLU 左为正）；图像上方 (dv<0) → +x 机头
        _, y, _, _ = vc.ellipse_to_body(960 + 100, 540, 100, 100, intr, 2.0, 0.0)
        self.assertAlmostEqual(y, -0.2, places=6)
        x, _, _, _ = vc.ellipse_to_body(960, 540 - 100, 100, 100, intr, 2.0, 0.0)
        self.assertAlmostEqual(x, 0.2, places=6)

    def test_mount_rot_90(self):
        intr = self.make_intr()
        # 装订旋转 90°：图像右方 → 机尾（-x）
        x, y, _, _ = vc.ellipse_to_body(960 + 100, 540, 100, 100, intr, 2.0,
                                        0.0, mount_rot_deg=90.0)
        self.assertAlmostEqual(x, -0.2, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)

    def test_anisotropic_fy(self):
        intr = self.make_intr(fx=FX, fy=FX / 2)
        # dv=100、fy=500 → x = 100·2/500 = 0.4；直径两轴折算平均
        x, _, _, d = vc.ellipse_to_body(960, 540 + 100, 100, 100, intr, 2.0, 0.0)
        self.assertAlmostEqual(x, -0.4, places=6)
        self.assertAlmostEqual(d, 0.5 * 2.0 * (100 / 1000 + 100 / 500), places=6)


class TestLabWhiteDetect(unittest.TestCase):

    def test_three_buckets_detected(self):
        h_m, img_w, img_h = 2.0, 640, 480
        frame = make_ground(img_w, img_h)
        truth = {0.15: (160, 120), 0.20: (400, 240), 0.25: (240, 380)}
        for diam, (cx, cy) in truth.items():
            draw_bucket(frame, cx, cy, diam * FX / h_m)
        params = vc.LabParams(diam_compensation=1.0)   # 硬边合成无腐蚀损耗
        dets = vc.lab_white_detect(frame, params)
        self.assertEqual(len(dets), 3, f'应检出 3 筒，实得 {len(dets)}')
        for det in dets:
            diam_px = (det.a_px + det.b_px) / 2.0
            expect = min(truth, key=lambda dm: abs(
                dm * FX / h_m - diam_px))
            self.assertAlmostEqual(diam_px, expect * FX / h_m, delta=8,
                                   msg=f'直径 {diam_px:.1f}px 偏离 {expect}m 真值')
            self.assertGreaterEqual(det.conf, 0.55)
            self.assertLessEqual(det.conf, 0.95)

    def test_empty_scene(self):
        dets = vc.lab_white_detect(make_ground(640, 480),
                                   vc.LabParams(diam_compensation=1.0))
        self.assertEqual(dets, [])

    def test_diam_compensation_inflates(self):
        frame = make_ground(640, 480)
        draw_bucket(frame, 320, 240, 100)
        d1 = vc.lab_white_detect(frame, vc.LabParams(diam_compensation=1.0))
        d2 = vc.lab_white_detect(frame, vc.LabParams(diam_compensation=1.15))
        self.assertEqual(len(d1), 1)
        self.assertEqual(len(d2), 1)
        self.assertAlmostEqual(d2[0].a_px, d1[0].a_px * 1.15, delta=1.0)


class TestHough(unittest.TestCase):

    FXH, HH = 1000.0, 1.5     # r_exp = 0.40·1000/1.5 ≈ 267px

    def test_h_circle_detected(self):
        frame = make_ground(800, 800)
        r = int(0.40 * self.FXH / self.HH)
        draw_h_circle(frame, 400, 400, r)
        dets = vc.hough_h_detect(frame, fx=self.FXH, h_m=self.HH)
        self.assertGreaterEqual(len(dets), 1, 'H 圆未检出')
        det = max(dets, key=lambda d: d.conf)
        self.assertAlmostEqual(det.a_px, 2 * r, delta=2 * r * 0.10)
        self.assertEqual(det.source, 'hough')
        self.assertGreaterEqual(det.conf, 0.55)

    def test_blank_ground_rejected(self):
        dets = vc.hough_h_detect(make_ground(800, 800), fx=self.FXH, h_m=self.HH)
        self.assertEqual(dets, [])

    def test_h_score_discriminates(self):
        gray_h = cv2.cvtColor(draw_h_circle(
            make_ground(800, 800, noise_sigma=0.0), 400, 400, 267),
            cv2.COLOR_BGR2GRAY)
        blank = make_ground(800, 800, noise_sigma=0.0)
        cv2.circle(blank, (400, 400), 267, (255, 255, 255), -1)
        gray_blank = cv2.cvtColor(blank, cv2.COLOR_BGR2GRAY)
        s_h = vc.h_pattern_score(gray_h, 400, 400, 267)
        s_blank = vc.h_pattern_score(gray_blank, 400, 400, 267)
        self.assertGreaterEqual(s_h, 0.45, f'H 图案得分过低 {s_h:.2f}')
        self.assertLess(s_blank, 0.45, f'空白圆得分虚高 {s_blank:.2f}')


class TestPostProcess(unittest.TestCase):

    @staticmethod
    def bd(x, y, diam, conf=0.8):
        return vc.BodyDet(x=x, y=y, z=-1.0, diam_m=diam, conf=conf)

    def test_merge_same_frame(self):
        a, b = self.bd(0, 0, 0.20, 0.9), self.bd(0.005, 0, 0.21, 0.6)
        out = vc.merge_same_frame([a, b])
        self.assertEqual(len(out), 1)
        self.assertAlmostEqual(out[0].diam_m, 0.20 + (0.21 - 0.20) * 0.6 / 1.5,
                               places=9)
        self.assertEqual(len(vc.merge_same_frame(
            [self.bd(0, 0, 0.2), self.bd(0.05, 0, 0.2)])), 2)

    def test_independence_conservative_and(self):
        near_same = [self.bd(0, 0, 0.15), self.bd(0.10, 0, 0.155)]
        self.assertEqual(len(vc.enforce_independence(near_same)), 1)
        near_diff = [self.bd(0, 0, 0.15), self.bd(0.10, 0, 0.20)]
        self.assertEqual(len(vc.enforce_independence(near_diff)), 2,
                         '近但直径异 → 感知端不合并（消费端终审）')
        far_same = [self.bd(0, 0, 0.15), self.bd(0.50, 0, 0.152)]
        self.assertEqual(len(vc.enforce_independence(far_same)), 2)
        # require_both=False 退化为消费端同款"或"判据
        self.assertEqual(len(vc.enforce_independence(far_same,
                                                     require_both=False)), 1)

    def test_nominal_bucket_id(self):
        # 容差含边界：0.115 恰好压线 → 归 0 号筒；再远一档才拒
        cases = {0.15: 0, 0.20: 1, 0.25: 2, 0.18: 1, 0.135: 0, 0.30: None,
                 0.112: None}
        for diam, want in cases.items():
            self.assertEqual(vc.nominal_bucket_id(diam), want,
                             f'{diam}m 对号错误')

    def test_apply_diameter_prior(self):
        dets = [self.bd(0, 0, 0.10), self.bd(1, 0, 0.05),
                self.bd(2, 0, 0.36), self.bd(3, 0, 0.20)]
        kept, rejected = vc.apply_diameter_prior(dets)
        self.assertEqual(rejected, 2)
        self.assertEqual(len(kept), 2)
        by_x = {round(d.x): d for d in kept}
        self.assertIsNone(by_x[0].bucket_class)     # 0.10m：保底保留但对不上号
        self.assertEqual(by_x[3].bucket_class, 1)   # 0.20m → 2 号筒


class TestFusion(unittest.TestCase):

    def test_matched_fused_conf(self):
        main = [vc.PixelDet(100, 100, 60, 60, 0.80, source='main')]
        aux = [vc.PixelDet(102, 100, 58, 58, 0.90, source='aux')]
        out = vc.fuse_channels(main, aux)
        self.assertEqual(len(out), 1)
        self.assertEqual(out[0].source, 'fused')
        self.assertAlmostEqual(out[0].conf,
                               min(0.98, 0.55 * 0.8 + 0.45 * 0.9 + 0.10),
                               places=9)
        # 几何取主通道
        self.assertAlmostEqual(out[0].u, 100.0)

    def test_unmatched_scales(self):
        main = [vc.PixelDet(100, 100, 60, 60, 0.80, source='main')]
        aux = [vc.PixelDet(400, 400, 60, 60, 0.80, source='aux')]
        out = sorted(vc.fuse_channels(main, aux), key=lambda d: d.source)
        by_src = {d.source: d for d in out}
        self.assertAlmostEqual(by_src['main'].conf, 0.72, places=9)
        self.assertAlmostEqual(by_src['aux'].conf, 0.56, places=9)


class TestMedianSmoother(unittest.TestCase):

    @staticmethod
    def bd(x, y, diam=0.20, conf=0.8):
        return vc.BodyDet(x=x, y=y, z=-1.0, diam_m=diam, conf=conf)

    def test_median_stabilizes(self):
        sm = vc.MedianSmoother(window=5)
        rng = np.random.default_rng(3)
        out = None
        for _ in range(6):
            out = sm.update([self.bd(1.0 + rng.normal(0, 0.02),
                                     1.0 + rng.normal(0, 0.02))])
        self.assertAlmostEqual(out[0].x, 1.0, delta=0.02)
        self.assertAlmostEqual(out[0].y, 1.0, delta=0.02)
        self.assertAlmostEqual(out[0].diam_m, 0.20, delta=0.005)

    def test_jump_creates_new_track(self):
        sm = vc.MedianSmoother(window=5)
        for _ in range(5):
            sm.update([self.bd(1.0, 1.0)])
        out = sm.update([self.bd(2.0, 1.0)])     # 跳变 1m > 门 0.25 → 新航迹
        self.assertAlmostEqual(out[0].x, 2.0, places=9, msg='新航迹不该被旧窗拖尾')

    def test_jitter_penalizes_conf(self):
        sm = vc.MedianSmoother(window=5)
        confs = []
        for i in range(6):
            x = 1.0 + (0.06 if i % 2 == 0 else -0.06)
            confs.append(sm.update([self.bd(x, 0.0)])[0].conf)
        self.assertLess(confs[-1], 0.8 * 0.9, '持续抖动未折减置信度')


class TestMisc(unittest.TestCase):

    def test_flu_to_frd(self):
        self.assertEqual(vc.flu_to_frd(1.0, 2.0, -3.0), (1.0, -2.0, 3.0))

    def test_bbox_iou(self):
        self.assertAlmostEqual(vc.bbox_iou((0, 0, 10, 10), (0, 0, 10, 10)), 1.0)
        self.assertAlmostEqual(vc.bbox_iou((0, 0, 10, 10), (20, 20, 30, 30)), 0.0)
        self.assertAlmostEqual(vc.bbox_iou((0, 0, 10, 10), (5, 0, 15, 10)),
                               50.0 / 150.0)

    def test_decode_jpeg_roundtrip(self):
        frame = make_ground(320, 240, noise_sigma=0.0)
        ok, buf = cv2.imencode('.jpg', frame)
        self.assertTrue(ok)
        back = vc.decode_jpeg(buf.tobytes())
        self.assertEqual(back.shape, (240, 320, 3))
        with self.assertRaises(ValueError):
            vc.decode_jpeg(b'not-jpeg')

    def test_constants_match_contract(self):
        # 与契约/SSOT 基线一致（改动需两侧同步评审）
        self.assertEqual(vc.DIAM_MIN_M, 0.08)
        self.assertEqual(vc.DIAM_MAX_M, 0.35)
        self.assertEqual(vc.BUCKET_NOMINALS_M, (0.15, 0.20, 0.25))
        self.assertEqual(vc.DIAM_NOMINAL_TOL_M, 0.035)
        self.assertEqual(vc.MAX_POSES_PER_FRAME, 8)


if __name__ == '__main__':
    unittest.main(verbosity=2)
