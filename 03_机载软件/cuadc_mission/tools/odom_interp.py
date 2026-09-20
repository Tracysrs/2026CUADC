"""odom_interp 的 Python 镜像（纯 stdlib）——P0.4 时间同步判定核心双端纪律。

C++ 权威实现 = include/cuadc_mission/odom_interp.hpp（09-20 自 mission_node
interpolate_odom 抽出，行为逐分支保持）；用例与 test/test_odom_interp.cpp
一一镜像（tools/test_odom_interp.py）。边界语义勿改：拒绝即丢弃，绝不外推。
"""

import math
from dataclasses import dataclass, field


def _normalize_angle(x: float) -> float:
    return math.atan2(math.sin(x), math.cos(x))


@dataclass
class Sample:
    t: float = 0.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0


@dataclass
class InterpParams:
    future_tol_s: float = 0.05   # 未来戳容差（SSOT §4.3）
    max_delay_s: float = 1.5     # 感知管线延迟上界
    max_gap_s: float = 0.2       # 插值最大帧间隔（断流段拒绝）


# 状态串（与 test 断言直接比对）：ok / empty / future_stamp / too_old / gap_break
OK = 'ok'
EMPTY = 'empty'
FUTURE_STAMP = 'future_stamp'
TOO_OLD = 'too_old'
GAP_BREAK = 'gap_break'


@dataclass
class InterpResult:
    status: str = EMPTY
    out: Sample = field(default_factory=Sample)


def interpolate(hist, t: float, p: InterpParams) -> InterpResult:
    """把时刻 t 夹逼/插值到 odom 历史上。边界语义（test_odom_interp 锁死）：
    空历史→empty；t 比最新超 future_tol（严格>）→future_stamp；t 比最老旧超
    max_delay（严格>）→too_old；t≥最新→最新（夹逼上界，恰等含）；t≤最老→
    最老（夹逼下界，恰等含）；包围两帧双闭、间隔>max_gap（严格>）→gap_break；
    否则线性插值 + yaw 最短角差。"""
    if not hist:
        return InterpResult(EMPTY)
    newest = hist[-1]
    oldest = hist[0]
    if t - newest.t > p.future_tol_s:
        return InterpResult(FUTURE_STAMP)
    if newest.t - t > p.max_delay_s:
        return InterpResult(TOO_OLD)
    if t >= newest.t:
        return InterpResult(OK, newest)
    if t <= oldest.t:
        return InterpResult(OK, oldest)
    for i in range(len(hist) - 1):
        a, b = hist[i], hist[i + 1]
        if a.t <= t <= b.t:
            gap = b.t - a.t
            if gap > p.max_gap_s:
                return InterpResult(GAP_BREAK)
            r = (t - a.t) / max(1e-6, gap)
            dyaw = _normalize_angle(b.yaw - a.yaw)
            return InterpResult(OK, Sample(
                t,
                a.x + r * (b.x - a.x), a.y + r * (b.y - a.y),
                a.z + r * (b.z - a.z), _normalize_angle(a.yaw + r * dyaw)))
    return InterpResult(GAP_BREAK)
