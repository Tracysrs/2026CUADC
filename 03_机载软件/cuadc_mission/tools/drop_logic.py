#!/usr/bin/env python3
"""投放决策核心（纯逻辑，无 ROS 依赖）—— M3 视觉投放算法的已验证参考实现。

链条（SSOT §5.2/§5.4/§6 + HIT 冠军版 cuadc_full_mission_node_3_v2_public 经验）：
  SEARCH   BucketMap：世界系检测 → 关联/EMA/确认 → 独立性强制合并 → 直径对号
           → 排名稳定 0.8s → 冻结目标集（冻结参考此后绝不改写）
  ALIGN    TargetTracker：维持活动估计（新鲜度 = 反盲投资格）；粗 0.15 → 精 0.08m；
           丢视觉只允许一次重捕获，再丢 = 弃桶拉黑（禁止按冻结坐标盲投）
  RELEASE  冻结瞄准点（冻结时刻的新鲜活动估计 + 标定投放口偏置 + 弹道前移）
           → 八门控全过且连续稳定 0.8s + 保持 1.5s（任一超时 6s 弃桶）
           → DropSequencer 单发舵机（fire 持续 0.7s 后回仓）

对应 HIT 教训的防线（每条都有单元测试，见 test_drop_logic.py）：
  - 区赛"两瓶投一点"（独立性判据写了没生效）→ enforce_independence() 真实合并
  - 区赛"按冻结坐标盲投"→ 门控内建 target_age 新鲜度 + 一次重捕获 + 弃桶
  - 舵机双发/卡死 → DropSequencer 单发状态机，二次 fire 直接拒绝
  - 排名抖动误锁 → 排名签名连续 0.8s 不变才放行
  - EMA 目标漂移 → 活动估计用 EMA，冻结参考永不被重捕获改写

坐标系约定：输入输出全部是**世界系**（本地 ENU，米）。视觉帧 → 世界系必须先经
mission_node 的 interpolate_odom() + body_to_local_at()（P0.4 取帧时刻插值），
本模块不做任何坐标变换，也不 import ROS——可离线单测/回放/蒙特卡洛。

飞行消费方：cuadc_mission（C++，M3 移植须与本模块逐函数对应，先过 SITL）。
闭环验证：drop_sim.py 蒙特卡洛（运动学 + 视觉噪声 + 舵机延迟）。
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from statistics import median
from typing import List, Optional, Sequence, Tuple

G_ACC = 9.81


def median_of(xs: Sequence[float]) -> float:
    return median(xs)


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


# ===========================================================================
# SEARCH 段：桶航迹地图 + 锁定五步
# ===========================================================================
@dataclass
class LockParams:
    """默认值 = SSOT §5.2/§5.4 基线；改参数走 YAML 注入，勿改死代码。"""

    min_confirm_frames: int = 5      # 锁定五步①：≥5 帧确认
    assoc_gate_m: float = 0.50       # 检测-航迹关联门（世界系；须 < 筒间距）
    ema_alpha: float = 0.25          # 位置 EMA（SSOT §5.2）
    diameter_window: int = 9         # 直径中位窗（SSOT §5.2）
    jitter_gate_m: float = 0.15      # 确认时位置抖动门（世界系 std）
    fresh_window_s: float = 1.0      # SEARCH 段航迹过期时间
    min_spacing_m: float = 0.20      # 独立性门控：水平距
    min_diameter_diff_m: float = 0.025
    diameter_min_m: float = 0.08     # 物理先验（免费一致性校验）
    diameter_max_m: float = 0.35
    nominal_diameters: Tuple[float, float, float] = (0.15, 0.20, 0.25)
    diameter_match_tol_m: float = 0.035
    rank_stable_s: float = 0.8       # 锁定五步⑤：排名稳定才锁
    preferred_targets: int = 3       # 正式模式需 3 独立分类筒
    degraded_classified: int = 2     # 降级模式（"2 筒 + 1 未知"）
    blacklist_radius_m: float = 0.25
    blacklist_window_s: float = 120.0


@dataclass
class BucketTrack:
    tid: int
    x: float
    y: float
    diameter: float                       # 中位数（无样本时 = 原始值）
    diam_samples: deque = field(default_factory=deque)
    positions: deque = field(default_factory=deque)   # 最近位置（算抖动）
    confirms: int = 1
    first_seen: float = 0.0
    last_seen: float = 0.0


@dataclass
class FrozenTarget:
    """冻结目标：frozen_* 锁死后绝不被改写；working_* 是活动估计。"""

    tid: int
    frozen_x: float
    frozen_y: float
    frozen_diameter: float
    diameter_class: int                   # 0/1/2 → 15/20/25cm；-1 = 未知
    working_x: float
    working_y: float
    last_vision_t: float
    reacquires_used: int = 0
    lost_since: Optional[float] = None


@dataclass
class LockResult:
    ok: bool
    reason: str
    targets: List[FrozenTarget] = field(default_factory=list)


class BucketMap:
    """SEARCH 段世界系桶航迹。检测须已换算到世界系（P0.4 插值之后）。"""

    def __init__(self, params: Optional[LockParams] = None):
        self.p = params or LockParams()
        self.tracks: List[BucketTrack] = []
        self._next_id = 0
        self.released: List[Tuple[float, float, float]] = []   # (x, y, t)
        self._rank_signature: Optional[Tuple] = None
        self._rank_since: Optional[float] = None

    # ---- 航迹维护 ----
    def update(self, x: float, y: float, diameter: float, t: float) -> str:
        """喂一帧（单个检测）。返回 'associated' / 'new_track' / 'rejected_diameter'。"""
        if not (self.p.diameter_min_m <= diameter <= self.p.diameter_max_m):
            return 'rejected_diameter'   # 物理先验：免费一致性校验，直接拒
        best: Optional[BucketTrack] = None
        best_d = self.p.assoc_gate_m
        for tr in self.tracks:
            d = math.hypot(tr.x - x, tr.y - y)
            if d <= best_d:
                best_d, best = d, tr
        if best is None:
            tr = BucketTrack(tid=self._next_id, x=x, y=y, diameter=diameter,
                             first_seen=t, last_seen=t)
            self._next_id += 1
            tr.positions.append((x, y))
            self.tracks.append(tr)
            return 'new_track'
        a = self.p.ema_alpha
        best.x = (1 - a) * best.x + a * x
        best.y = (1 - a) * best.y + a * y
        best.diam_samples.append(diameter)
        while len(best.diam_samples) > self.p.diameter_window:
            best.diam_samples.popleft()
        best.diameter = median_of(best.diam_samples)
        best.positions.append((best.x, best.y))
        while len(best.positions) > 10:
            best.positions.popleft()
        best.confirms += 1
        best.last_seen = t
        return 'associated'

    # ---- 确认与独立性 ----
    def fresh(self, t: float) -> List[BucketTrack]:
        return [tr for tr in self.tracks
                if t - tr.last_seen <= self.p.fresh_window_s]

    @staticmethod
    def _jitter(tr: BucketTrack) -> float:
        if len(tr.positions) < 2:
            return float('inf')
        xs = [p[0] for p in tr.positions]
        ys = [p[1] for p in tr.positions]
        return max(_std(xs), _std(ys))

    def confirmed(self, t: float) -> List[BucketTrack]:
        out = []
        for tr in self.fresh(t):
            if tr.confirms < self.p.min_confirm_frames:
                continue
            if self._jitter(tr) > self.p.jitter_gate_m:
                continue
            out.append(tr)
        return out

    def enforce_independence(self, t: float) -> int:
        """独立性门控（真实生效版）：两确认航迹 间距<0.20m 或 直径差<0.025m
        → 判为同一筒，强制合并（保留确认数多者）。返回合并次数。
        这是 HIT 区赛"两瓶投一点"的直接防线——判据在这里真实参与删除。"""
        changed = True
        merges = 0
        while changed:
            changed = False
            trs = self.confirmed(t)
            for i in range(len(trs)):
                for j in range(i + 1, len(trs)):
                    a, b = trs[i], trs[j]
                    dist = math.hypot(a.x - b.x, a.y - b.y)
                    ddiff = abs(a.diameter - b.diameter)
                    if dist < self.p.min_spacing_m or ddiff < self.p.min_diameter_diff_m:
                        keep, drop = (a, b) if a.confirms >= b.confirms else (b, a)
                        self.tracks.remove(drop)
                        merges += 1
                        changed = True
                        break
                if changed:
                    break
        return merges

    # ---- 直径对号 ----
    def diameter_class(self, d: float) -> Optional[int]:
        best, best_err = None, self.p.diameter_match_tol_m
        for i, nom in enumerate(self.p.nominal_diameters):
            err = abs(d - nom)
            if err <= best_err:
                best, best_err = i, err
        return best

    # ---- 锁定（锁定五步⑤：排名稳定才冻结）----
    def try_lock(self, t: float, degraded: bool = False) -> LockResult:
        self.enforce_independence(t)
        trs = [tr for tr in self.confirmed(t)
               if not self.is_blacklisted(tr.x, tr.y, t)]   # 已投/已弃筒绝不回锁
        classified, unknown = [], []
        for tr in trs:
            cls = self.diameter_class(tr.diameter)
            (classified if cls is not None else unknown).append((tr, cls))
        need = self.p.degraded_classified if degraded else self.p.preferred_targets
        if len(classified) < need:
            return LockResult(False, f'classified {len(classified)} < {need}')
        if not degraded and unknown:
            unknown = []            # 正式模式：未知筒不入选（宁缺勿错）
        # 排序：直径类升序（aggressive 语义：先小后大）；未知排最后
        classified.sort(key=lambda tc: (tc[1], tc[0].diameter))
        ordered = classified + unknown
        signature = tuple(tr.tid for tr, _ in ordered)
        if signature != self._rank_signature:
            self._rank_signature = signature
            self._rank_since = t
            return LockResult(False, 'rank_changed')
        if self._rank_since is None or t - self._rank_since < self.p.rank_stable_s:
            return LockResult(False, 'rank_not_stable')
        targets = [
            FrozenTarget(
                tid=tr.tid, frozen_x=tr.x, frozen_y=tr.y,
                frozen_diameter=tr.diameter, diameter_class=cls,
                working_x=tr.x, working_y=tr.y, last_vision_t=t)
            for tr, cls in ordered
        ]
        return LockResult(True, f'locked {len(targets)} targets', targets)

    # ---- 拉黑 ----
    def blacklist(self, x: float, y: float, t: float) -> None:
        self.released.append((x, y, t))

    def is_blacklisted(self, x: float, y: float, t: float) -> bool:
        for rx, ry, rt in self.released:
            if (math.hypot(x - rx, y - ry) <= self.p.blacklist_radius_m
                    and t - rt <= self.p.blacklist_window_s):
                return True
        return False


def _std(xs: Sequence[float]) -> float:
    n = len(xs)
    if n < 2:
        return 0.0
    m = sum(xs) / n
    return math.sqrt(sum((x - m) ** 2 for x in xs) / n)


# ===========================================================================
# ALIGN/RELEASE 段：活动估计维护 + 反盲投重捕获策略
# ===========================================================================
@dataclass
class TrackParams:
    fresh_window_s: float = 0.5        # 目标估计新鲜度（反盲投：超龄禁投）
    ema_alpha: float = 0.25
    reacquire_pos_gate_m: float = 0.40
    reacquire_diam_gate_m: float = 0.06
    reacquire_window_s: float = 2.0    # 丢视觉后允许等待重捕获的时长
    max_reacquires: int = 1            # SSOT：丢视觉重捕获一次，仍丢 = 弃桶


class TargetTracker:
    """单个冻结目标的活动估计器。update() 每帧喂世界系检测（全部检测，
    由本类按门限自选；多目标各自实例独立维护，靠小门限 + 直径门防串）。"""

    def __init__(self, target: FrozenTarget, params: Optional[TrackParams] = None):
        self.target = target
        self.p = params or TrackParams()

    def update(self, detections: Sequence[Tuple[float, float, float]], t: float) -> bool:
        """detections = [(x, y, diameter), ...]（世界系）。返回是否关联成功。"""
        best, best_d = None, self.p.reacquire_pos_gate_m
        for x, y, d in detections:
            if abs(d - self.target.frozen_diameter) > self.p.reacquire_diam_gate_m:
                continue
            dist = math.hypot(x - self.target.working_x, y - self.target.working_y)
            if dist <= best_d:
                best_d, best = dist, (x, y)
        if best is None:
            return False
        a = self.p.ema_alpha
        self.target.working_x = (1 - a) * self.target.working_x + a * best[0]
        self.target.working_y = (1 - a) * self.target.working_y + a * best[1]
        self.target.last_vision_t = t
        self.target.lost_since = None
        return True

    def vision_age(self, t: float) -> float:
        return t - self.target.last_vision_t

    def assess(self, t: float) -> str:
        """'ok' = 新鲜；'reacquiring' = 丢视觉、重捕获窗口内（禁投）；
        'abandon' = 重捕获机会已用完/窗口耗尽 → 弃桶。"""
        lost = self.vision_age(t) > self.p.fresh_window_s
        if not lost:
            return 'ok'
        if self.target.reacquires_used >= self.p.max_reacquires:
            return 'abandon'
        since = self.target.lost_since
        if since is None:
            self.target.lost_since = t
            return 'reacquiring'
        if t - since > self.p.reacquire_window_s:
            self.target.reacquires_used = self.p.max_reacquires   # 窗口耗尽 = 机会用掉
            return 'abandon'
        return 'reacquiring'

    def on_reacquired(self) -> None:
        """重捕获成功后由 mission 调用：扣次数（机会只有一次）。"""
        self.target.reacquires_used += 1


# ===========================================================================
# RELEASE 段：八门控 + 弹道前移 + 单发舵机
# ===========================================================================
@dataclass
class GateParams:
    """默认值 = SSOT §5.2 释放八门控 + §5.4 基线。"""

    max_horizontal_error_m: float = 0.10
    max_vertical_error_m: float = 0.10
    max_hspeed_m_s: float = 0.08
    max_vspeed_m_s: float = 0.05
    max_tilt_deg: float = 5.0
    max_yaw_err_deg: float = 5.0
    stability_s: float = 0.8            # 连续无违约稳定
    hold_s: float = 1.5                 # 达到稳定后再保持（合计连续 2.3s）
    timeout_s: float = 6.0              # 进入 RELEASE 起 6s 未凑齐 → 弃桶
    max_target_age_s: float = 0.5       # 反盲投：目标估计超龄禁投
    max_lead_m: float = 0.15            # 弹道前移 sanity 上限（悬停应 ≈0）
    release_height_m: float = 1.8       # 释放高度（对准高度，SSOT 粗/精对准 1.8m）


@dataclass
class GateSample:
    horiz_err_m: float
    vert_err_m: float
    hspeed_m_s: float
    vspeed_m_s: float
    tilt_deg: float
    yaw_err_deg: float
    target_age_s: float
    vx_m_s: float                        # 世界系速度（弹道前移 sanity 用）
    vy_m_s: float
    t: float


def ballistic_lead(vx: float, vy: float, height_m: float,
                   g: float = G_ACC) -> Tuple[float, float]:
    """弹道前移 Δ = v·√(2h/g)。悬停时 ≈0，保留作 sanity check（SSOT §5.2）。"""
    t_fall = math.sqrt(2.0 * max(0.0, height_m) / g)
    return vx * t_fall, vy * t_fall


class ReleaseGate:
    """八门控评估器。feed() 每拍一次；返回 ('idle'|'holding'|'fire'|'fired'|'abort', 原因)。
    'fire' 只出现一次，此后恒 'fired'（幂等）；任一项违约清零两个计时器。"""

    def __init__(self, params: Optional[GateParams] = None):
        self.p = params or GateParams()
        self.reset()

    def reset(self) -> None:
        self.pass_started: Optional[float] = None
        self.entered_t: Optional[float] = None
        self.fired = False
        self.aborted = False
        self.abort_reason = ''

    def _instant(self, s: GateSample) -> Tuple[bool, str]:
        vals = [s.horiz_err_m, s.vert_err_m, s.hspeed_m_s, s.vspeed_m_s,
                s.tilt_deg, s.yaw_err_deg, s.target_age_s, s.t]
        if not all(math.isfinite(v) for v in vals):
            return False, 'non_finite'
        if s.target_age_s > self.p.max_target_age_s:
            return False, 'target_stale'          # 反盲投核心防线
        if s.horiz_err_m > self.p.max_horizontal_error_m:
            return False, 'horizontal_error'
        if abs(s.vert_err_m) > self.p.max_vertical_error_m:
            return False, 'vertical_error'
        if s.hspeed_m_s > self.p.max_hspeed_m_s:
            return False, 'horizontal_speed'
        if abs(s.vspeed_m_s) > self.p.max_vspeed_m_s:
            return False, 'vertical_speed'
        if s.tilt_deg > self.p.max_tilt_deg:
            return False, 'tilt'
        if s.yaw_err_deg > self.p.max_yaw_err_deg:
            return False, 'yaw_error'
        lead_x, lead_y = ballistic_lead(s.vx_m_s, s.vy_m_s, self.p.release_height_m)
        if math.hypot(lead_x, lead_y) > self.p.max_lead_m:
            return False, 'lead_excess'           # sanity：速度门限失守时的第二道闸
        return True, 'ok'

    def feed(self, s: GateSample) -> Tuple[str, str]:
        if self.fired:
            return 'fired', 'already_fired'
        if self.aborted:
            return 'abort', self.abort_reason   # 保留原始弃桶原因，排障用
        if self.entered_t is None:
            self.entered_t = s.t
        ok, reason = self._instant(s)
        if not ok:
            self.pass_started = None
            if s.t - self.entered_t > self.p.timeout_s:
                self.aborted = True
                self.abort_reason = f'timeout_last_reason={reason}'
                return 'abort', self.abort_reason
            return 'idle', reason
        if self.pass_started is None:
            self.pass_started = s.t
        held = s.t - self.pass_started
        if held >= self.p.stability_s + self.p.hold_s:
            self.fired = True
            return 'fire', 'gates_satisfied'
        if s.t - self.entered_t > self.p.timeout_s:
            self.aborted = True
            self.abort_reason = 'timeout'
            return 'abort', self.abort_reason
        return 'holding', f'held={held:.2f}s'


class DropSequencer:
    """舵机单发状态机：IDLE →（fire）RELEASED →（hold 到时）STOWED。
    fire() 二次调用返回 False——双发保护是硬性的，不靠调用方自觉。"""

    HOLD_S = 0.7                          # SSOT §5.4：释放保持 0.7s 后回仓

    def __init__(self):
        self.phase = 'IDLE'
        self.fire_t: Optional[float] = None

    def fire(self, t: float) -> bool:
        if self.phase != 'IDLE':
            return False
        self.phase = 'RELEASED'
        self.fire_t = t
        return True

    def tick(self, t: float) -> Optional[str]:
        """返回 'stow'（恰好一次）表示该回仓；其余时刻返回 None。"""
        if self.phase == 'RELEASED' and self.fire_t is not None \
                and t - self.fire_t >= self.HOLD_S:
            self.phase = 'STOWED'
            return 'stow'
        return None


# ===========================================================================
# 瞄准点解算（冻结 + 标定偏置 + 弹道前移）
# ===========================================================================
def aim_point(frozen: FrozenTarget, offset_body_xy: Tuple[float, float],
              mission_yaw: float, vx: float, vy: float,
              params: GateParams) -> Tuple[float, float, float, float]:
    """瞄准点 = 冻结时刻的活动估计 + 标定投放口偏置（机体系→世界，用锁定航向）
    + 弹道前移（带 sanity 钳位）。返回 (ax, ay, lead_x, lead_y)。
    标定偏置来源：SSOT §6 两步法（静态铅垂线中位数 + 试投中位数修正 + MAD）。"""
    c, s = math.cos(mission_yaw), math.sin(mission_yaw)
    ox = c * offset_body_xy[0] - s * offset_body_xy[1]
    oy = s * offset_body_xy[0] + c * offset_body_xy[1]
    lx, ly = ballistic_lead(vx, vy, params.release_height_m)
    norm = math.hypot(lx, ly)
    if norm > params.max_lead_m and norm > 0:
        lx, ly = lx * params.max_lead_m / norm, ly * params.max_lead_m / norm
    return frozen.working_x + ox + lx, frozen.working_y + oy + ly, lx, ly
