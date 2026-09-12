#!/usr/bin/env python3
"""投放链闭环仿真 —— 用蒙特卡洛证明 drop_logic 决策核心能准确投放。

模型（故意简化的部分，均已注明）：
  - 飞机：水平一阶速度响应 + 位置目标追踪（真实 ArduPilot 位置环的保守近似）；
  - 视觉：20Hz 检测，世界系残差 σ=2.5cm（取帧插值 P0.4 之后的残差——插值本身
    已由 时间同步设计.md 单独验收，此处不重复建模）、漏检概率、可注入全局断视觉窗；
  - 舵机：fire 后 0.1s 动作延迟（SSOT：指令到动作 <200ms）；
  - 瓶：落点 = 释放位置 + 释放速度 × 落地时间 + 标定残差散布（σ=2cm）；
    未命中也如实记录，不筛选成功案例（HIT 提案2 的做法）。

验证的不变量（每集检查，任一违反记入 violations）：
  I1 舵机每载荷至多 fire 一次（DropSequencer 双发保护）
  I2 fire 当拍目标新鲜度 ≤ max_target_age（反盲投）
  I3 两次成功投放必为不同筒（冻结集 + 拉黑）
  I4 弃桶(abandon)后该筒绝不再被投
  I5 断视觉超重捕获预算 → 必须走弃桶，不得开火

用法：
    python -m unittest discover -s cuadc_mission/tools   # 含本文件测试
    python -c "import sys; sys.path.insert(0,'cuadc_mission/tools');
               from drop_sim import *;
               rs=run_monte_carlo(30); print(sum(r.locked for r in rs),'/',len(rs))"
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from drop_logic import (
    BucketMap,
    DropSequencer,
    FrozenTarget,
    GateParams,
    GateSample,
    LockParams,
    ReleaseGate,
    TargetTracker,
    TrackParams,
    aim_point,
)

NOMINAL = (0.15, 0.20, 0.25)
RADIUS = (0.075, 0.10, 0.125)          # 桶口半径（15/20/25cm）
G_ACC = 9.81


@dataclass
class SimParams:
    seed: int = 2026
    dt: float = 0.05                    # 状态机拍 = 视觉帧 20Hz
    sigma_world_m: float = 0.025        # 时间同步补偿后的视觉残差
    p_detect: float = 0.95
    kp: float = 1.2
    tau: float = 0.35
    search_vmax: float = 2.0            # SSOT §5.4 搜索速度
    align_vmax: float = 0.7             # 投放进近速度
    release_vmax: float = 0.25          # 释放定位速度
    wind_clamp: float = 0.05
    dispersion_m: float = 0.02          # 舵机延迟抖动 + 标定残差
    actuation_delay_s: float = 0.1
    align_alt_m: float = 1.8            # SSOT 粗/精对准高度
    align_timeout_s: float = 12.0       # SSOT §5.4 对准超时
    coarse_radius_m: float = 0.15
    fine_radius_m: float = 0.08
    align_stable_s: float = 0.8
    mission_yaw: float = 0.0            # 锁定航向（全程锁头不转机头）
    payload_offset_body: Tuple[float, float] = (0.03, 0.01)   # §6 标定偏置
    n_payloads: int = 2                 # 比赛 = 两瓶（1 号筒 + 2 号筒）；仿真可调 3 做压力
    drop_order: str = 'conservative'    # conservative=先大后小 / aggressive
    t_max_s: float = 200.0


@dataclass
class Bucket:
    cid: int
    x: float
    y: float
    diameter_class: int                 # 0/1/2

    @property
    def diameter(self) -> float:
        return NOMINAL[self.diameter_class]

    @property
    def radius(self) -> float:
        return RADIUS[self.diameter_class]


@dataclass
class Landing:
    diameter_class: int
    error_m: float
    hit: bool


@dataclass
class EpisodeResult:
    locked: bool = False
    fired_classes: List[int] = field(default_factory=list)    # 成功投放的筒类
    abandoned_classes: List[int] = field(default_factory=list)
    landings: List[Landing] = field(default_factory=list)
    violations: List[str] = field(default_factory=list)
    end_state: str = ''
    t_end: float = 0.0

    @property
    def n_fired(self) -> int:
        return len(self.fired_classes)

    @property
    def hit_rate(self) -> float:
        return (sum(1 for l in self.landings if l.hit) / len(self.landings)
                if self.landings else 0.0)

    @property
    def cep(self) -> float:
        errs = sorted(l.error_m for l in self.landings)
        if not errs:
            return float('inf')
        n = len(errs)
        return errs[n // 2] if n % 2 else 0.5 * (errs[n // 2 - 1] + errs[n // 2])


def _sorted_targets(targets: List[FrozenTarget], order: str) -> List[FrozenTarget]:
    """conservative=直径类降序（先大筒保底）；aggressive=升序（先小筒冲 800 分）。
    未知类（-1）排最后。"""
    known = [t for t in targets if t.diameter_class >= 0]
    unknown = [t for t in targets if t.diameter_class < 0]
    known.sort(key=lambda t: t.diameter_class, reverse=(order == 'conservative'))
    return known + unknown


def _waypoint(lane_ys: List[float], idx: int) -> Tuple[float, float]:
    """弓字形：偶数段朝 +x，奇数段朝 -x（起飞点 (0,0)，搜索区 x∈[6,10]）。"""
    lane = min(idx // 2, len(lane_ys) - 1)
    x = 10.0 if idx % 2 == 0 else 6.0
    return x, lane_ys[lane]


def _pursue(x: float, vx: float, y: float, vy: float, tx: float, ty: float,
            vmax: float, p: SimParams) -> Tuple[float, float, float, float, bool]:
    """位置目标追踪：速度指令 = kp·误差（限幅）+ 一阶速度响应。"""
    dx, dy = tx - x, ty - y
    dist = math.hypot(dx, dy)
    sp = p.kp * dist
    if dist > 1e-9:
        k = min(1.0, sp / dist)
        cvx, cvy = dx * k, dy * k
    else:
        cvx = cvy = 0.0
    mag = math.hypot(cvx, cvy)
    if mag > vmax:
        cvx, cvy = cvx * vmax / mag, cvy * vmax / mag
    a = p.dt / p.tau
    vx += (cvx - vx) * a
    vy += (cvy - vy) * a
    x += vx * p.dt
    y += vy * p.dt
    return x, vx, y, vy, math.hypot(tx - x, ty - y) <= 0.25


def run_episode(params: SimParams,
                blackout: Optional[Tuple[float, float]] = None) -> EpisodeResult:
    """blackout = (t0, t1)：该窗口内全局无检测（断视觉场景，验证 I5）。"""
    rng = random.Random(params.seed)
    res = EpisodeResult()

    # ---- 场景：三筒随机摆放，间距 ≥0.8m（> 2×关联门的一半，防串迹）----
    buckets: List[Bucket] = []
    classes = [0, 1, 2]
    rng.shuffle(classes)
    while len(buckets) < 3:
        x = rng.uniform(6.5, 9.5)
        y = rng.uniform(-1.5, 1.5)
        if all(math.hypot(x - b.x, y - b.y) >= 0.8 for b in buckets):
            buckets.append(Bucket(cid=len(buckets), x=x, y=y,
                                  diameter_class=classes[len(buckets)]))

    # ---- 运行时状态 ----
    lock_p, track_p, gate_p = LockParams(), TrackParams(), GateParams()
    bmap = BucketMap(lock_p)
    ac_x, ac_y, ac_vx, ac_vy = 0.0, 0.0, 0.0, 0.0
    wind_x = wind_y = 0.0
    t = 0.0
    state = 'SEARCH'
    lane_ys = [-2.0, 2.0, -1.0, 1.0]
    wp_idx = 0
    payload_order: List[FrozenTarget] = []
    payload_idx = -1
    tracker: Optional[TargetTracker] = None
    align_stage = 'coarse'
    align_since: Optional[float] = None
    align_t0 = 0.0
    aim = (0.0, 0.0)
    gate: Optional[ReleaseGate] = None
    seq: Optional[DropSequencer] = None
    fire_recorded = False
    bottle_release_t: Optional[float] = None
    fired_tids: set = set()
    current_tid: Optional[int] = None

    # ---- 嵌套帮助函数（必须在主循环之前定义）----
    def detections(t_now: float) -> List[Tuple[float, float, float]]:
        if blackout and blackout[0] <= t_now <= blackout[1]:
            return []
        out = []
        for b in buckets:
            if math.hypot(b.x - ac_x, b.y - ac_y) > 6.0:
                continue
            if rng.random() > params.p_detect:
                continue
            out.append((b.x + rng.gauss(0, params.sigma_world_m),
                        b.y + rng.gauss(0, params.sigma_world_m),
                        b.diameter))
        return out

    def wind_step() -> None:
        nonlocal wind_x, wind_y, ac_x, ac_y
        wind_x = max(-params.wind_clamp, min(params.wind_clamp,
                     wind_x + rng.gauss(0, 0.004)))
        wind_y = max(-params.wind_clamp, min(params.wind_clamp,
                     wind_y + rng.gauss(0, 0.004)))
        ac_x += wind_x * params.dt
        ac_y += wind_y * params.dt

    def drop_bottle() -> None:
        """弹道：落点 = 释放位置 + 速度 × 落地时间 + 散布。对真值筒算误差。
        真值匹配必须按位置最近——track tid 是发现顺序编号，≠ 场景 cid。"""
        assert tracker is not None
        t_fall = math.sqrt(2.0 * params.align_alt_m / G_ACC)
        lx = ac_x + ac_vx * t_fall + rng.gauss(0, params.dispersion_m)
        ly = ac_y + ac_vy * t_fall + rng.gauss(0, params.dispersion_m)
        tg = tracker.target
        truth = min(buckets, key=lambda b: math.hypot(b.x - tg.frozen_x,
                                                      b.y - tg.frozen_y))
        err = math.hypot(lx - truth.x, ly - truth.y)
        res.landings.append(Landing(truth.diameter_class, err, err <= truth.radius))

    def next_payload(t_now: float) -> bool:
        """取下一个未拉黑目标；没有则任务收尾。"""
        nonlocal payload_idx, tracker, state, align_stage, align_since, align_t0
        nonlocal gate, seq, fire_recorded, bottle_release_t, current_tid
        payload_idx += 1
        while payload_idx < len(payload_order):
            if payload_idx >= params.n_payloads:
                break                       # 瓶投完了（比赛两瓶）
            tg = payload_order[payload_idx]
            if not bmap.is_blacklisted(tg.frozen_x, tg.frozen_y, t_now):
                tracker = TargetTracker(tg, track_p)
                state, align_stage = 'ALIGN', 'coarse'
                align_since, align_t0 = None, t_now
                gate, seq = None, None
                fire_recorded = False
                bottle_release_t = None
                current_tid = tg.tid
                return True
            payload_idx += 1
        state = 'DONE'
        res.end_state = 'mission_complete'
        return False

    def abandon_target(t_now: float) -> None:
        """弃桶拉黑（I4 防线）。"""
        assert tracker is not None
        res.abandoned_classes.append(tracker.target.diameter_class)
        bmap.blacklist(tracker.target.frozen_x, tracker.target.frozen_y, t_now)
        next_payload(t_now)

    # ---- 主循环 ----
    while state != 'DONE' and t < params.t_max_s:
        dets = detections(t)

        if state == 'SEARCH':
            for x, y, d in dets:
                bmap.update(x, y, d, t)
            tx, ty = _waypoint(lane_ys, wp_idx)
            ac_x, ac_vx, ac_y, ac_vy, arrived = _pursue(
                ac_x, ac_vx, ac_y, ac_vy, tx, ty, params.search_vmax, params)
            wind_step()
            if arrived:
                wp_idx += 1
            if wp_idx >= 2 * len(lane_ys):
                res.end_state = 'search_no_lock'
                break
            lock = bmap.try_lock(t)
            if lock.ok:
                payload_order = _sorted_targets(lock.targets, params.drop_order)
                payload_idx = -1
                res.locked = True
                if not next_payload(t):
                    break

        elif state == 'ALIGN':
            assert tracker is not None
            tracker.update(dets, t)
            if tracker.assess(t) == 'abandon':                    # I4/I5 防线
                abandon_target(t)
                continue
            tg = tracker.target
            err = math.hypot(ac_x - tg.working_x, ac_y - tg.working_y)
            ac_x, ac_vx, ac_y, ac_vy, _ = _pursue(
                ac_x, ac_vx, ac_y, ac_vy, tg.working_x, tg.working_y,
                params.align_vmax, params)
            wind_step()
            radius = (params.coarse_radius_m if align_stage == 'coarse'
                      else params.fine_radius_m)
            if err <= radius:
                if align_stage == 'coarse':                       # 粗 → 精
                    align_stage = 'fine'
                    align_since = None
                elif align_since is None:
                    align_since = t
                if (align_stage == 'fine' and align_since is not None
                        and t - align_since >= params.align_stable_s):
                    # 冻结瞄准点：新鲜活动估计 + 标定偏置 + 弹道前移（SSOT §5.2/§6）
                    aim = aim_point(tg, params.payload_offset_body,
                                    params.mission_yaw, ac_vx, ac_vy, gate_p)[:2]
                    gate, seq = ReleaseGate(gate_p), DropSequencer()
                    fire_recorded = False
                    bottle_release_t = None
                    state = 'RELEASE'
            else:
                align_since = None
            if t - align_t0 > params.align_timeout_s:
                abandon_target(t)
                continue

        elif state == 'RELEASE':
            assert tracker is not None and gate is not None and seq is not None
            tracker.update(dets, t)                               # 释放段持续复核
            age = tracker.vision_age(t)
            hs = math.hypot(ac_vx, ac_vy)
            sample = GateSample(
                horiz_err_m=math.hypot(ac_x - aim[0], ac_y - aim[1]),
                vert_err_m=0.0, hspeed_m_s=hs, vspeed_m_s=0.0,
                tilt_deg=1.0 + 40.0 * hs, yaw_err_deg=0.0,
                target_age_s=age, vx_m_s=ac_vx, vy_m_s=ac_vy, t=t)
            status, _ = gate.feed(sample)
            ac_x, ac_vx, ac_y, ac_vy, _ = _pursue(
                ac_x, ac_vx, ac_y, ac_vy, aim[0], aim[1],
                params.release_vmax, params)
            wind_step()

            if status == 'fire':
                # ---- 不变量 I1/I2/I3 ----
                if fire_recorded or not seq.fire(t):
                    res.violations.append(f'I1_double_fire t={t:.2f}')
                if age > gate_p.max_target_age_s:
                    res.violations.append(f'I2_stale_fire t={t:.2f}')
                if current_tid in fired_tids:
                    res.violations.append(f'I3_same_bucket t={t:.2f}')
                fire_recorded = True
                fired_tids.add(current_tid)
                bottle_release_t = t + params.actuation_delay_s   # 舵机动作延迟
            if bottle_release_t is not None and t >= bottle_release_t:
                drop_bottle()
                bottle_release_t = None
            if status == 'abort':                                 # I5：超时弃桶
                abandon_target(t)
                continue
            if seq.tick(t) == 'stow':                             # 保持 0.7s 到
                bmap.blacklist(tracker.target.frozen_x, tracker.target.frozen_y, t)
                res.fired_classes.append(tracker.target.diameter_class)
                if not next_payload(t):
                    break

        t += params.dt

    if state == 'DONE' or not res.end_state:
        res.end_state = res.end_state or ('timeout' if state != 'DONE' else res.end_state)
    res.t_end = t
    return res


def run_monte_carlo(n: int, base_seed: int = 2026,
                    blackout: Optional[Tuple[float, float]] = None,
                    **overrides) -> List[EpisodeResult]:
    out = []
    for i in range(n):
        p = SimParams(seed=base_seed + i, **overrides)
        out.append(run_episode(p, blackout=blackout))
    return out
