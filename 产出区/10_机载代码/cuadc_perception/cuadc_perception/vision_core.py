#!/usr/bin/env python3
"""vision_core — 视觉纯算法库（无 ROS 依赖，Windows 单测 / Jetson 节点共用）。

设计权威：产出区/01_设计/视觉算法设计.md；契约：10_机载代码/接口契约.md v1.3。
判据与消费端 drop_logic.py 同源（SSOT §4.1/§4.2/§5.4 基线），改动必须两边同步评审。

模块分工（对应设计文档 §3/§4）：
  Intrinsics          相机内参（标定 yaml / datasheet FOV 兜底）+ 正交性校验
  ellipse_to_body     单目解算：像素椭圆 → 机体系 (FLU)，主用 odom 相对高度
  lab_white_detect    白桶副通道：LAB 相对阈值 + 形态学 + fitEllipse（仿真已验证）
  hough_h_detect      H 圆兜底通道：动态半径窗 HoughCircles + H 图案验证打分
  fuse_channels       主/副双通道 IoU 融合（主=YOLO seg，副=LAB）
  merge_same_frame    同帧去重（中心距 <2cm 合并）
  enforce_independence  独立性门控（感知端保守版：距离与直径差须同时过线才合并）
  apply_diameter_prior  直径物理先验 [0.08,0.35]m + 对号 15/20/25cm（拒绝原因计数）
  MedianSmoother      滑窗中位数平滑（不确认——确认权在消费端 BucketMap）
  flu_to_frd          FLU→FRD 换算（LANDING_TARGET 用，y/z 取反）

约定：所有几何在"相机离目标平面的高度 h_m"下解算（目标平面：桶口 0.30m / H 圆 0m），
SSOT §4.2 的 Z=f·H/h_px 仅在边缘斜视角可用（正下视中心区域无筒侧壁），降级为 sanity。
"""

from __future__ import annotations

import hashlib
import math
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# 物理常量（SSOT §5.4 基线，与 cuadc_mission drop_logic 同源）
# ---------------------------------------------------------------------------
DIAM_MIN_M = 0.08           # 契约直径物理下界（check_vision_contract 同款）
DIAM_MAX_M = 0.35           # 契约直径物理上界
BUCKET_NOMINALS_M = (0.15, 0.20, 0.25)   # 1/2/3 号筒口直径
DIAM_NOMINAL_TOL_M = 0.035  # 对号容差（drop_logic 同款）
H_CIRCLE_DIAM_M = 0.80      # 起降区 H 圆直径（细则 3.1.1）
BUCKET_TOP_Z_M = 0.30       # 投放筒高（桶口平面）
MAX_POSES_PER_FRAME = 8     # 契约单帧上限


class CalibrationError(RuntimeError):
    """内参缺失/正交性校验失败（工程五件套之三：错误阻断启动）。"""


# ---------------------------------------------------------------------------
# 相机内参
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Intrinsics:
    """针孔内参。datasheet 兜底来源 = 规格书视场角推导，dist 视为零。"""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    dist_coeffs: Tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0)
    source: str = 'datasheet'   # 'calib' | 'datasheet'
    reproj_error_px: float = 0.0

    @classmethod
    def from_mapping(cls, m: dict, default_width: int,
                     default_height: int) -> 'Intrinsics':
        """从标定 yaml 的键值构造（fx/fy/cx/cy 必填；分辨率取 yaml 记载值，
        缺失时退回当前帧尺寸——cx/cy 的中心带校验依赖正确的分辨率）。"""
        try:
            dist = tuple(float(v) for v in m.get('dist_coeffs', (0.0,) * 5))
            return cls(
                fx=float(m['fx']), fy=float(m['fy']),
                cx=float(m['cx']), cy=float(m['cy']),
                width=int(m.get('image_width', default_width)),
                height=int(m.get('image_height', default_height)),
                dist_coeffs=dist,
                source=str(m.get('source', 'calib')),
                reproj_error_px=float(m.get('reproj_error_px', 0.0)),
            )
        except KeyError as e:
            raise CalibrationError(f'标定文件缺字段 {e}（需要 fx/fy/cx/cy）') from None

    @classmethod
    def from_fov(cls, width: int, height: int, hfov_rad: float,
                 vfov_rad: Optional[float] = None) -> 'Intrinsics':
        """由视场角推导（BL-500W-335：H81.8°/V66° → 1080p 下 fx≈1112/fy≈831）。

        vfov 缺省时按方像素 fy=fx 处理（4:3 全幅）；16:9 裁剪必须同时给 H/V 两个
        视场角，否则 fy 被高估 ~34%（2026-09 调研发现，勿省）。
        """
        fx = (width / 2.0) / math.tan(hfov_rad / 2.0)
        fy = fx if vfov_rad is None else (height / 2.0) / math.tan(vfov_rad / 2.0)
        return cls(fx=fx, fy=fy, cx=width / 2.0, cy=height / 2.0,
                   width=width, height=height, source='datasheet')

    def validate(self) -> None:
        """正交性校验（工程五件套之三）：异常抛 CalibrationError，阻断启动。"""
        errs = []
        if self.fx <= 1.0 or self.fy <= 1.0:
            errs.append(f'焦距非正 fx={self.fx:.1f} fy={self.fy:.1f}')
        if self.width > 0 and self.height > 0:
            if not (0.3 * self.width <= self.cx <= 0.7 * self.width):
                errs.append(f'cx={self.cx:.0f} 偏离画面中心带 [{0.3*self.width:.0f},{0.7*self.width:.0f}]')
            if not (0.3 * self.height <= self.cy <= 0.7 * self.height):
                errs.append(f'cy={self.cy:.0f} 偏离画面中心带')
        ratio = self.fy / max(self.fx, 1e-6)
        if not (0.5 <= ratio <= 2.0):
            errs.append(f'fy/fx={ratio:.2f} 越界 [0.5,2.0]（16:9 裁剪合理值 ~0.75）')
        if len(self.dist_coeffs) not in (0, 4, 5, 8):
            errs.append(f'畸变系数长度 {len(self.dist_coeffs)} 非法')
        if self.source == 'calib' and abs(self.dist_coeffs[0]) > 0.5:
            errs.append(f'k1={self.dist_coeffs[0]:.2f} 过大，疑似标定失败')
        if errs:
            raise CalibrationError('内参校验失败: ' + '; '.join(errs))

    def scaled_to(self, width: int, height: int) -> 'Intrinsics':
        """等比缩放到处理分辨率（畸变系数近似不变，仅在降采样通道用）。"""
        if (width, height) == (self.width, self.height):
            return self
        sx, sy = width / self.width, height / self.height
        return Intrinsics(fx=self.fx * sx, fy=self.fy * sy, cx=self.cx * sx,
                          cy=self.cy * sy, width=width, height=height,
                          dist_coeffs=self.dist_coeffs, source=self.source,
                          reproj_error_px=self.reproj_error_px)


def load_intrinsics(calib_path: Optional[str], width: int, height: int,
                    hfov_rad: float = math.radians(81.8),
                    vfov_rad: float = math.radians(66.0),
                    allow_uncalibrated: bool = True) -> Tuple[Intrinsics, List[str]]:
    """加载内参：calib yaml 优先，缺失/禁用时按 datasheet FOV 兜底。

    返回 (内参, 告警列表)；比赛模式 allow_uncalibrated=False 且无标定文件 → 抛错。
    """
    import os
    warnings: List[str] = []
    if calib_path and os.path.isfile(calib_path):
        import yaml
        with open(calib_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        intr = Intrinsics.from_mapping(data, width, height)
        if (intr.width, intr.height) != (width, height):
            warnings.append(
                f'标定分辨率 {intr.width}x{intr.height} ≠ 当前 {width}x{height}，'
                f'已按比例缩放（大畸变边缘精度下降，建议按当前分辨率重标）')
            intr = intr.scaled_to(width, height)
        intr.validate()
        return intr, warnings
    if not allow_uncalibrated:
        raise CalibrationError(
            f'未找到标定文件 {calib_path} 且 allow_uncalibrated=False（比赛模式必须先标定）')
    warnings.append(f'无标定文件({calib_path})，按 datasheet FOV 推内参——直径/位置精度受限，'
                    f'外场前必须完成棋盘格标定（scripts/calibrate_camera.py）')
    intr = Intrinsics.from_fov(width, height, hfov_rad, vfov_rad)
    intr.validate()
    return intr, warnings


def sha256_of_file(path: str, chunk: int = 1 << 20) -> str:
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        while True:
            b = f.read(chunk)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


# ---------------------------------------------------------------------------
# 像素级检测
# ---------------------------------------------------------------------------
@dataclass
class PixelDet:
    """像素空间检测：椭圆 (u, v, a, b) + 置信度。Hough 圆 a=b=2r。"""

    u: float
    v: float
    a_px: float                 # 长轴（像素）
    b_px: float                 # 短轴（像素）
    conf: float
    source: str = 'aux'         # 'main' | 'aux' | 'hough' | 'fused'
    score_extra: float = 0.0    # 圆度 / H 验证分（日志与调试图用）

    @property
    def bbox(self) -> Tuple[float, float, float, float]:
        return (self.u - self.a_px / 2.0, self.v - self.b_px / 2.0,
                self.u + self.a_px / 2.0, self.v + self.b_px / 2.0)


def bbox_iou(b1: Sequence[float], b2: Sequence[float]) -> float:
    x0 = max(b1[0], b2[0]); y0 = max(b1[1], b2[1])
    x1 = min(b1[2], b2[2]); y1 = min(b1[3], b2[3])
    w, h = max(0.0, x1 - x0), max(0.0, y1 - y0)
    inter = w * h
    if inter <= 0.0:
        return 0.0
    a1 = (b1[2] - b1[0]) * (b1[3] - b1[1])
    a2 = (b2[2] - b2[0]) * (b2[3] - b2[1])
    return inter / max(1e-9, a1 + a2 - inter)


# ---------------------------------------------------------------------------
# LAB 白桶分割（副通道，SSOT §4.1；参数沿用仿真节点 09-12 标定值）
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class LabParams:
    l_min: float = 160.0        # L 兜底下限（8bit LAB）
    l_offset: float = 10.0      # 相对阈值：L > 帧中位数 + offset（抗渲染/光照差异）
    ab_max_dev: float = 22.0    # A/B 离中性最大偏差（近白；蓝地 B 通道远离中性被排除）
    min_area_px: float = 40.0   # 连通域最小面积（处理分辨率下）
    min_circularity: float = 0.60   # 椭圆短/长轴比下限（正下视近圆）
    diam_compensation: float = 1.15  # 阈值+腐蚀致椭圆偏小的补偿系数（09-12 标定）
    conf_base: float = 0.55
    conf_circ_gain: float = 0.45


def lab_white_detect(bgr: np.ndarray, params: LabParams = LabParams(),
                     max_dets: int = MAX_POSES_PER_FRAME) -> List[PixelDet]:
    """LAB 白桶分割 → 椭圆拟合。返回像素空间候选（未解算、未过直径先验）。"""
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    L = lab[:, :, 0].astype(np.float32)
    A = lab[:, :, 1].astype(np.float32)
    B = lab[:, :, 2].astype(np.float32)
    l_thr = max(params.l_min, float(np.median(L)) + params.l_offset)
    mask = ((L > l_thr) &
            (np.abs(A - 128.0) < params.ab_max_dev) &
            (np.abs(B - 128.0) < params.ab_max_dev)).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, (5, 5))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out: List[PixelDet] = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < params.min_area_px or len(cnt) < 5:
            continue
        (eu, ev), (d_a, d_b), _ang = cv2.fitEllipse(cnt)
        if d_a <= 1.0 or d_b <= 1.0:
            continue
        circularity = min(d_a, d_b) / max(d_a, d_b)
        if circularity < params.min_circularity:
            continue
        conf = min(0.95, params.conf_base + params.conf_circ_gain * circularity)
        out.append(PixelDet(u=eu, v=ev, a_px=d_a * params.diam_compensation,
                            b_px=d_b * params.diam_compensation,
                            conf=conf, source='aux', score_extra=circularity))
        if len(out) >= max_dets:
            break
    return out


# ---------------------------------------------------------------------------
# H 圆兜底通道：HoughCircles + H 图案验证
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class HoughParams:
    radius_tol: float = 0.35        # 动态半径窗 ±35%
    dp: float = 1.0
    param1: float = 120.0           # Canny 高阈值
    param2: float = 40.0            # 累加器阈值（大圆 30~60，过严漏检）
    h_score_min: float = 0.45       # H 图案验证下限（0 = 关闭验证）
    blur_ksize: int = 5


def h_pattern_score(gray: np.ndarray, u: float, v: float, r: float) -> float:
    """圆内 H 图案验证打分 [0,1]。

    假设标志制式 = 白圆 + 黑 H（自制采集卡按此制作，对比度最大）。
    特征：Otsu 后取少数派掩码（H），左/右竖带能量接近（对称）且显著。
    这是软验证非分类——误拒由时序确认窗兜底，误收由半径-高度一致性门把守。
    """
    x0, y0 = int(u - r), int(v - r)
    x1, y1 = int(u + r), int(v + r)
    h, w = gray.shape[:2]
    x0, y0 = max(0, x0), max(0, y0)
    x1, y1 = min(w, x1), min(h, y1)
    if x1 - x0 < 8 or y1 - y0 < 8:
        return 0.0
    crop = gray[y0:y1, x0:x1]
    _, bw = cv2.threshold(crop, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    # 少数派 = H（白圆底上黑 H → 前景少；若反色同样适用）
    if np.count_nonzero(bw) > bw.size // 2:
        bw = cv2.bitwise_not(bw)
    m = bw.astype(np.float32) / 255.0
    cx = crop.shape[1] / 2.0
    cy = crop.shape[0] / 2.0
    r_eff = min(cx, cy) * 0.9
    xs = np.arange(crop.shape[1])[None, :].repeat(crop.shape[0], 0) - cx
    ys = np.arange(crop.shape[0])[:, None].repeat(crop.shape[1], 1) - cy
    inside = (xs ** 2 + ys ** 2) <= r_eff ** 2
    band_l = inside & (np.abs(xs) >= 0.30 * r_eff) & (np.abs(xs) <= 0.80 * r_eff)
    band_c = inside & (np.abs(xs) < 0.25 * r_eff) & (np.abs(ys) <= 0.33 * r_eff)
    area_l = max(1, int(np.count_nonzero(band_l)))
    area_c = max(1, int(np.count_nonzero(band_c)))
    # 左右竖带各分正负 x 两叶
    leaf_area = max(1, area_l // 2)
    e_lp = float(m[band_l & (xs > 0)].sum()) / leaf_area
    e_ln = float(m[band_l & (xs < 0)].sum()) / leaf_area
    e_c = float(m[band_c].sum()) / area_c
    # 存在性门：竖带几乎无图案（空白圆/纯色圆）直接归零——防对称性白送分数
    presence = min(1.0, (e_lp + e_ln) / 0.30)
    if presence <= 0.0:
        return 0.0
    sym = 1.0 - abs(e_lp - e_ln) / max(0.05, e_lp + e_ln)
    bars = min(e_lp, e_ln) / 0.45          # 每叶竖杠期望填充率 ~0.45（杠宽/叶宽）
    bars = max(0.0, min(1.0, bars))
    return max(0.0, min(1.0, (0.5 * sym + 0.5 * bars) * presence))


def hough_h_detect(bgr: np.ndarray, fx: float, h_m: float,
                   params: HoughParams = HoughParams(),
                   h_diam_m: float = H_CIRCLE_DIAM_M,
                   max_dets: int = 3) -> List[PixelDet]:
    """H 圆 Hough 检测：半径窗由高度动态推算（r = 0.40·fx/h），越界拒检。"""
    if h_m <= 0.05:
        return []
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    k = max(3, params.blur_ksize | 1)
    blur = cv2.GaussianBlur(gray, (k, k), 0)
    r_exp = (h_diam_m / 2.0) * fx / h_m
    r_lo = int(max(6.0, r_exp * (1.0 - params.radius_tol)))
    r_hi = int(min(min(gray.shape[:2]) / 2.0 - 1, r_exp * (1.0 + params.radius_tol)))
    if r_lo >= r_hi:
        return []
    circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT, dp=params.dp,
                               minDist=max(30.0, r_exp),
                               param1=params.param1, param2=params.param2,
                               minRadius=r_lo, maxRadius=r_hi)
    out: List[PixelDet] = []
    if circles is None:
        return out
    for u, v, r in circles[0][:max_dets * 2]:
        score = 1.0
        if params.h_score_min > 0.0:
            score = h_pattern_score(gray, float(u), float(v), float(r))
            if score < params.h_score_min:
                continue
        out.append(PixelDet(u=float(u), v=float(v), a_px=2.0 * float(r),
                            b_px=2.0 * float(r),
                            conf=max(0.55, 0.55 * score + 0.40),
                            source='hough', score_extra=score))
        if len(out) >= max_dets:
            break
    return out


# ---------------------------------------------------------------------------
# 单目解算（主用 odom 相对高度）
# ---------------------------------------------------------------------------
def ellipse_to_body(u: float, v: float, a_px: float, b_px: float,
                    intr: Intrinsics, h_m: float, plane_z_m: float,
                    mount_rot_deg: float = 0.0) -> Tuple[float, float, float, float]:
    """像素椭圆 → 机体系 (FLU)。

    h_m = 相机离目标平面的高度（= odom_z − plane_z_m）；返回 z = −h_m（目标在下方）。
    mount_rot_deg = 装订旋转：图像 (du,dv) 先旋转再投影（默认 0 = 图像上=机头、
    图像左=机体左）。首次悬停偏置目标实验标定（设计文档 §4.6）。
    直径：两轴分别按各自焦距折算再平均（16:9 下 fx≠fy）。
    """
    du, dv = u - intr.cx, v - intr.cy
    if mount_rot_deg:
        du, dv = rotate2d(du, dv, mount_rot_deg)
    x = -dv * h_m / intr.fy            # 图像上方 = 机头（默认装订）
    y = -du * h_m / intr.fx            # 图像左方 = 机体左
    z = -(h_m)
    diam_m = 0.5 * h_m * (a_px / intr.fx + b_px / intr.fy)
    return x, y, z, diam_m


def rotate2d(du: float, dv: float, deg: float) -> Tuple[float, float]:
    th = math.radians(deg)
    c, s = math.cos(th), math.sin(th)
    return du * c - dv * s, du * s + dv * c


def flu_to_frd(x: float, y: float, z: float) -> Tuple[float, float, float]:
    """FLU (x前 y左 z上) → FRD (x前 y右 z下)：y、z 取反。LANDING_TARGET 专用陷阱。"""
    return x, -y, -z


# ---------------------------------------------------------------------------
# 机体系检测与后处理四件（SSOT §4.1；消费端 drop_logic 终审）
# ---------------------------------------------------------------------------
@dataclass
class BodyDet:
    x: float
    y: float
    z: float
    diam_m: float
    conf: float
    source: str = 'fused'
    bucket_class: Optional[int] = None   # 0/1/2 = 15/20/25cm；None = 未对号
    u: float = 0.0
    v: float = 0.0
    reject_reason: str = ''


def merge_same_frame(dets: List[BodyDet],
                     merge_dist_m: float = 0.02) -> List[BodyDet]:
    """同帧去重：中心距 <2cm 合并（HIT"同桶拆两目标"防线；消费端另有终审）。

    合并取置信高者的位置/类别，直径按置信加权平均。
    """
    kept: List[BodyDet] = []
    for d in sorted(dets, key=lambda t: -t.conf):
        hit = None
        for k in kept:
            if math.hypot(k.x - d.x, k.y - d.y) < merge_dist_m:
                hit = k
                break
        if hit is None:
            kept.append(d)
        else:
            wsum = hit.conf + d.conf
            if wsum > 0:
                hit.diam_m = (hit.diam_m * hit.conf + d.diam_m * d.conf) / wsum
            hit.source = hit.source if hit.source == d.source else 'fused'
    return kept


def enforce_independence(dets: List[BodyDet], min_dist_m: float = 0.20,
                         min_diam_diff_m: float = 0.025,
                         require_both: bool = True) -> List[BodyDet]:
    """独立性门控（感知端保守版）。

    消费端 BucketMap 判据 = 距离 <0.20 **或** 直径差 <0.025 即合并（规则保证三筒
    直径互异，可放心）；感知端在**源头**上多合并会误杀真筒且无从恢复，故默认
    require_both=True：两条件同时过线（近且同粗）才判同筒，其余留给消费端终审。
    """
    kept: List[BodyDet] = []
    for d in sorted(dets, key=lambda t: -t.conf):
        dup = False
        for k in kept:
            near = math.hypot(k.x - d.x, k.y - d.y) < min_dist_m
            same = abs(k.diam_m - d.diam_m) < min_diam_diff_m
            violated = (near and same) if require_both else (near or same)
            if violated:
                dup = True
                break
        if not dup:
            kept.append(d)
    return kept


def diameter_prior_ok(diam_m: float, lo: float = DIAM_MIN_M,
                      hi: float = DIAM_MAX_M) -> bool:
    return lo <= diam_m <= hi


def nominal_bucket_id(diam_m: float, nominals: Sequence[float] = BUCKET_NOMINALS_M,
                      tol: float = DIAM_NOMINAL_TOL_M) -> Optional[int]:
    """对号 15/20/25cm：取容差内**最近**者（0.18m 应归 0.20 而非 0.15）。"""
    best, best_diff = None, tol
    for i, nom in enumerate(nominals):
        diff = abs(diam_m - nom)
        if diff <= best_diff:
            best, best_diff = i, diff
    return best


def apply_diameter_prior(dets: List[BodyDet]) -> Tuple[List[BodyDet], int]:
    """直径物理先验门控：[0.08,0.35]m 内保留并附对号类别，越界拒检。

    返回 (过检列表, 拒绝数)。拒绝是免费的物理一致性校验（SSOT §4.1）——
    0.8m 起飞坪大白块、场地标线反光在此被挡在门外。
    """
    kept: List[BodyDet] = []
    rejected = 0
    for d in dets:
        if not diameter_prior_ok(d.diam_m):
            d.reject_reason = f'diam_out_of_range({d.diam_m:.3f}m)'
            rejected += 1
            continue
        d.bucket_class = nominal_bucket_id(d.diam_m)
        kept.append(d)
    return kept, rejected


# ---------------------------------------------------------------------------
# 双通道融合（主 YOLO-seg / 副 LAB）
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class FusionParams:
    iou_thr: float = 0.5
    w_main: float = 0.55
    w_aux: float = 0.45
    agree_bonus: float = 0.10   # 双通道独立证据一致 → 置信奖励
    main_only_scale: float = 0.90
    aux_only_scale: float = 0.70  # 仅副通道命中 = 光照兜底，保守打折
    conf_max: float = 0.98


def fuse_channels(main: List[PixelDet], aux: List[PixelDet],
                  params: FusionParams = FusionParams()) -> List[PixelDet]:
    """IoU 匹配融合。几何取主通道（seg 掩码更精），置信按通道加权+一致奖励。"""
    out: List[PixelDet] = []
    aux_used = [False] * len(aux)
    for m in sorted(main, key=lambda t: -t.conf):
        mb = m.bbox
        best_i, best_iou = -1, params.iou_thr
        for i, a in enumerate(aux):
            if aux_used[i]:
                continue
            ov = bbox_iou(mb, a.bbox)
            if ov >= best_iou:
                best_i, best_iou = i, ov
        if best_i >= 0:
            aux_used[best_i] = True
            a = aux[best_i]
            conf = min(params.conf_max,
                       params.w_main * m.conf + params.w_aux * a.conf + params.agree_bonus)
            out.append(PixelDet(u=m.u, v=m.v, a_px=m.a_px, b_px=m.b_px,
                                conf=conf, source='fused',
                                score_extra=max(m.score_extra, a.score_extra)))
        else:
            out.append(PixelDet(u=m.u, v=m.v, a_px=m.a_px, b_px=m.b_px,
                                conf=min(params.conf_max, m.conf * params.main_only_scale),
                                source='main', score_extra=m.score_extra))
    for i, a in enumerate(aux):
        if not aux_used[i]:
            out.append(PixelDet(u=a.u, v=a.v, a_px=a.a_px, b_px=a.b_px,
                                conf=min(params.conf_max, a.conf * params.aux_only_scale),
                                source='aux', score_extra=a.score_extra))
    return out


# ---------------------------------------------------------------------------
# 时序平滑（中位数窗；不确认——确认权在消费端 BucketMap，避免双重确认叠加延迟）
# ---------------------------------------------------------------------------
class MedianSmoother:
    """贪心最近邻关联 + 滑窗中位数（位置/直径）。

    窗内位置 std 折入置信度（抖动惩罚）；未关联上的新目标直通不拖尾。
    关联门 0.25m < 规则最小筒间距判据的邻档，误串仅影响平滑不影响身份
    （身份/确认/拉黑全部由消费端 BucketMap 负责）。
    """

    def __init__(self, window: int = 5, assoc_gate_m: float = 0.25,
                 max_tracks: int = 16, stale_frames: int = 6,
                 jitter_std_ok_m: float = 0.02, jitter_std_bad_m: float = 0.10,
                 jitter_conf_penalty: float = 0.50):
        self.window = max(1, window)
        self.gate = assoc_gate_m
        self.max_tracks = max_tracks
        self.stale_frames = stale_frames
        self.ok = jitter_std_ok_m
        self.bad = max(jitter_std_bad_m, jitter_std_ok_m + 1e-3)
        self.penalty = jitter_conf_penalty
        self._hist: List[deque] = []
        self._age: List[int] = []
        self._next_id = 0
        self._ids: List[int] = []

    def _new_track(self) -> int:
        if len(self._hist) >= self.max_tracks:
            oldest = max(range(len(self._hist)), key=lambda i: self._age[i])
            self._hist.pop(oldest); self._age.pop(oldest); self._ids.pop(oldest)
        self._hist.append(deque(maxlen=self.window))
        self._age.append(0)
        self._ids.append(self._next_id)
        self._next_id += 1
        return len(self._hist) - 1

    def update(self, dets: List[BodyDet]) -> List[BodyDet]:
        matched_tracks: set = set()
        out: List[BodyDet] = []
        for d in sorted(dets, key=lambda t: -t.conf):
            best_i, best_d = -1, self.gate
            for i, hist in enumerate(self._hist):
                if i in matched_tracks or not hist:
                    continue
                dist = math.hypot(hist[-1][0] - d.x, hist[-1][1] - d.y)
                if dist <= best_d:
                    best_i, best_d = i, dist
            if best_i < 0:
                best_i = self._new_track()
            matched_tracks.add(best_i)
            self._age[best_i] = 0
            self._hist[best_i].append((d.x, d.y, d.diam_m))
            hist = self._hist[best_i]
            xs = np.array([p[0] for p in hist]); ys = np.array([p[1] for p in hist])
            ds = np.array([p[2] for p in hist])
            jitter = math.hypot(float(xs.std()), float(ys.std()))
            smooth = BodyDet(
                x=float(np.median(xs)), y=float(np.median(ys)), z=d.z,
                diam_m=float(np.median(ds)), conf=d.conf, source=d.source,
                bucket_class=d.bucket_class, u=d.u, v=d.v)
            if len(hist) >= 3 and jitter > self.ok:
                ratio = min(1.0, (jitter - self.ok) / (self.bad - self.ok))
                smooth.conf = max(0.05, d.conf * (1.0 - self.penalty * ratio))
            out.append(smooth)
        for i in range(len(self._age)):
            if i not in matched_tracks:
                self._age[i] += 1
        # 清理失联超限的航迹（倒序删避免索引位移）
        for i in range(len(self._age) - 1, -1, -1):
            if self._age[i] > self.stale_frames:
                self._hist.pop(i); self._age.pop(i); self._ids.pop(i)
        return out


# ---------------------------------------------------------------------------
# 小工具
# ---------------------------------------------------------------------------
def decode_jpeg(data: bytes) -> np.ndarray:
    """JPEG 字节 → BGR（相机 MJPG 透传帧；失败抛 ValueError 由节点按丢帧处理）。"""
    arr = np.frombuffer(data, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError('JPEG 解码失败（截断帧？）')
    return img


def clamp_conf(c: float, lo: float = 0.0, hi: float = 0.98) -> float:
    return max(lo, min(hi, float(c)))
