"""侦察判读融合逻辑（纯函数库，无 ROS 依赖）。

三层结构（阈值默认值 = SSOT §4.4，逐类标定后由离线评估工具产出覆盖值）：
  1. 帧级门控 FrameGate：top1 < 0.7 或 top1-top2 < 0.3 → 丢弃；
     灰区（top1 ∈ [0.7,0.85) 或 margin ∈ [0.3,0.45)）→ need_boost，
     由调用方做 ROI 全分辨率放大重推后用 check_boosted() 复查（只查硬门槛）。
  2. 标记级融合 MarkerFusion：同一融合窗口内按框 IoU 关联同一物理标识，
     累积各类票数与置信度历史。
  3. 结论规则 verdicts()：某类 票数≥5 且 置信中位数≥0.8 且 票数≥3×次高类
     → 确认；两类同时达标、或达标但无主导 → ambiguous（留空）；
     无任何类达标 → 留空（class_id=-1）。
  计分语义：留空 0 分，错填 -100 → 一切阈值以"错填率≈0"优先。

本模块刻意不 import rclpy/传感器消息：单元测试与本机（Windows）可直接运行；
真节点（M4）与离线评估工具都只消费这里的数据类与规则。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from statistics import median
from typing import Dict, List

# ---- 帧级门控默认阈值（SSOT §4.4；逐类标定后覆盖）----
REJECT_TOP1 = 0.7          # top1 硬门槛
REJECT_MARGIN = 0.3        # top1-top2 硬门槛
GRAY_TOP1_HIGH = 0.85      # 灰区上界：[REJECT_TOP1, GRAY_TOP1_HIGH) 需 ROI 放大重推
GRAY_MARGIN_HIGH = 0.45

# ---- 标记级结论规则默认值（HIT 的连续 5 帧经验 + 计分语义收紧）----
CONFIRM_MIN_FRAMES = 5
CONFIRM_MIN_MEDIAN_CONF = 0.8
CONFIRM_VOTE_RATIO = 3.0   # 主类票数必须 ≥ 比例 × 次高类票数

CLASS_ID_BLANK = -1        # 拒识留空


@dataclass
class Detection:
    """一次单帧检测（已经过坐标还原，全分辨率像素坐标系）。

    confidence=top1，runner_up_conf=top2（无次高类时填 0.0）；
    stamp_s = 该帧取帧时刻（秒，与感知端 header.stamp 同源）。
    """

    u: float
    v: float
    w: float
    h: float
    class_id: int
    confidence: float
    runner_up_conf: float
    stamp_s: float

    @property
    def margin(self) -> float:
        return self.confidence - self.runner_up_conf


@dataclass
class GateResult:
    accepted: bool       # 是否计入标记级投票
    need_boost: bool     # 灰区：调用方可 ROI 放大重推一次
    reason: str          # ok / low_confidence / low_margin / gray_confidence / gray_margin


class FrameGate:
    """帧级门控。阈值全部可注入（离线标定扫 τ 用同一份代码）。"""

    def __init__(
        self,
        reject_top1: float = REJECT_TOP1,
        reject_margin: float = REJECT_MARGIN,
        gray_top1_high: float = GRAY_TOP1_HIGH,
        gray_margin_high: float = GRAY_MARGIN_HIGH,
    ):
        if not (0.0 < reject_top1 <= gray_top1_high <= 1.0):
            raise ValueError('要求 0 < reject_top1 <= gray_top1_high <= 1')
        if not (0.0 < reject_margin <= gray_margin_high <= 1.0):
            raise ValueError('要求 0 < reject_margin <= gray_margin_high <= 1')
        self.reject_top1 = reject_top1
        self.reject_margin = reject_margin
        self.gray_top1_high = gray_top1_high
        self.gray_margin_high = gray_margin_high

    def check(self, det: Detection) -> GateResult:
        return self._check(det, boosted=False)

    def check_boosted(self, det: Detection) -> GateResult:
        """ROI 放大重推后的复查：灰区视为通过，只再查硬门槛。"""
        return self._check(det, boosted=True)

    def _check(self, det: Detection, boosted: bool) -> GateResult:
        if det.confidence < self.reject_top1:
            return GateResult(False, False, 'low_confidence')
        if det.margin < self.reject_margin:
            return GateResult(False, False, 'low_margin')
        gray = False
        reason = 'ok'
        if det.confidence < self.gray_top1_high:
            gray, reason = True, 'gray_confidence'
        if det.margin < self.gray_margin_high:
            gray, reason = True, 'gray_margin' if reason == 'ok' else 'gray_confidence+gray_margin'
        if gray and not boosted:
            return GateResult(False, True, reason)
        return GateResult(True, False, 'ok' if not gray else 'ok_after_boost')


@dataclass
class MarkerAccumulator:
    """一个物理标识的跨帧累积器（窗口内按 IoU 关联）。"""

    marker_index: int
    last_u: float
    last_v: float
    last_w: float
    last_h: float
    votes: Dict[int, int] = field(default_factory=dict)
    conf_history: Dict[int, List[float]] = field(default_factory=dict)
    margin_history: Dict[int, List[float]] = field(default_factory=dict)
    last_stamp_s: float = 0.0

    def add(self, det: Detection) -> None:
        self.votes[det.class_id] = self.votes.get(det.class_id, 0) + 1
        self.conf_history.setdefault(det.class_id, []).append(det.confidence)
        self.margin_history.setdefault(det.class_id, []).append(det.margin)
        self.last_u, self.last_v, self.last_w, self.last_h = det.u, det.v, det.w, det.h
        self.last_stamp_s = det.stamp_s


@dataclass
class MarkerVerdict:
    """一个物理标识的窗口结论（对应 ReconMarker.msg 的载荷语义）。"""

    marker_index: int
    class_id: int              # 0..9；-1 = 留空
    confidence: float          # 中位置信度；留空时给出最强类的中位数供显示
    margin: float
    frames: int                # 主类（或最强类）票数
    ambiguous: bool            # 类别竞争导致拒识
    insufficient: bool         # 帧数/置信不足导致留空


def bbox_iou(a_u: float, a_v: float, a_w: float, a_h: float,
             b_u: float, b_v: float, b_w: float, b_h: float) -> float:
    """中心+宽高 表示的两框 IoU（像素坐标）。"""

    ax1, ay1, ax2, ay2 = a_u - a_w / 2, a_v - a_h / 2, a_u + a_w / 2, a_v + a_h / 2
    bx1, by1, bx2, by2 = b_u - b_w / 2, b_v - b_h / 2, b_u + b_w / 2, b_v + b_h / 2
    iw = min(ax2, bx2) - max(ax1, bx1)
    ih = min(ay2, by2) - max(ay1, by1)
    if iw <= 0.0 or ih <= 0.0:
        return 0.0
    inter = iw * ih
    union = a_w * a_h + b_w * b_h - inter
    return inter / union if union > 0.0 else 0.0


class MarkerFusion:
    """窗口级融合器：一个 RECON 视点/悬停窗口一个实例，窗口切换时 reset()。

    update() 只消费通过帧级门控的检测（门控职责在 FrameGate，不在本类）。
    """

    def __init__(
        self,
        assoc_iou: float = 0.3,
        min_frames: int = CONFIRM_MIN_FRAMES,
        min_median_conf: float = CONFIRM_MIN_MEDIAN_CONF,
        vote_ratio: float = CONFIRM_VOTE_RATIO,
    ):
        if not (0.0 < assoc_iou < 1.0):
            raise ValueError('assoc_iou 必须在 (0,1)')
        self.assoc_iou = assoc_iou
        self.min_frames = min_frames
        self.min_median_conf = min_median_conf
        self.vote_ratio = vote_ratio
        self.markers: List[MarkerAccumulator] = []

    def reset(self) -> None:
        self.markers.clear()

    def update(self, detections: List[Detection]) -> None:
        for det in detections:
            acc = self._associate(det)
            if acc is None:
                acc = MarkerAccumulator(
                    marker_index=len(self.markers),
                    last_u=det.u, last_v=det.v, last_w=det.w, last_h=det.h,
                )
                self.markers.append(acc)
            acc.add(det)

    def _associate(self, det: Detection) -> MarkerAccumulator | None:
        best, best_iou = None, self.assoc_iou
        for acc in self.markers:
            iou = bbox_iou(det.u, det.v, det.w, det.h,
                           acc.last_u, acc.last_v, acc.last_w, acc.last_h)
            if iou >= best_iou:
                best, best_iou = acc, iou
        return best

    def verdicts(self) -> List[MarkerVerdict]:
        return [self._verdict(acc) for acc in self.markers]

    def _verdict(self, acc: MarkerAccumulator) -> MarkerVerdict:
        ranked = sorted(acc.votes.items(), key=lambda kv: kv[1], reverse=True)
        # 最强类：按票数，平票取置信中位数高者（供留空时的显示值）
        best_class = max(
            acc.votes,
            key=lambda c: (acc.votes[c], median(acc.conf_history[c])),
        )
        best_median = median(acc.conf_history[best_class])
        best_margin = median(acc.margin_history[best_class])

        # 达标类 = 票数与置信都过硬门槛的类
        qualified = [
            c for c, n in acc.votes.items()
            if n >= self.min_frames and median(acc.conf_history[c]) >= self.min_median_conf
        ]
        if len(qualified) > 1:
            return MarkerVerdict(acc.marker_index, CLASS_ID_BLANK,
                                 best_median, best_margin, acc.votes[best_class],
                                 ambiguous=True, insufficient=False)
        if len(qualified) == 1:
            c = qualified[0]
            runner_up_votes = max((n for other, n in acc.votes.items() if other != c),
                                  default=0)
            if acc.votes[c] >= self.vote_ratio * runner_up_votes:
                return MarkerVerdict(acc.marker_index, c,
                                     median(acc.conf_history[c]),
                                     median(acc.margin_history[c]),
                                     acc.votes[c],
                                     ambiguous=False, insufficient=False)
            # 达标但无主导（如 5:4）→ 竞争拒识
            return MarkerVerdict(acc.marker_index, CLASS_ID_BLANK,
                                 best_median, best_margin, acc.votes[best_class],
                                 ambiguous=True, insufficient=False)
        # 无类达标：帧数或置信不足 → 留空（不算混淆）
        return MarkerVerdict(acc.marker_index, CLASS_ID_BLANK,
                             best_median, best_margin, acc.votes[best_class],
                             ambiguous=False, insufficient=True)
