"""drop_logic 单元测试（纯 stdlib，本机直接跑，不需要 ROS）：

    cd 03_机载软件
    python -m unittest discover -s cuadc_mission/tools -v

每条 HIT 教训对应的防线都必须有对应用例——防线没有测试 = 防线不存在。
"""

import math
import unittest

from drop_logic import (
    BucketMap,
    BucketTrack,
    DropSequencer,
    FrozenTarget,
    GateParams,
    GateSample,
    LockParams,
    ReleaseGate,
    TargetTracker,
    TrackParams,
    aim_point,
    ballistic_lead,
)


def feed(map_, xs, t0=0.0, dt=0.05, diameter=0.20, jitter=0.0):
    """按序列喂检测，返回最后时刻。xs = [(x, y), ...]"""
    t = t0
    for x, y in xs:
        map_.update(x + jitter, y, diameter, t)
        t += dt
    return t - dt


class TestBucketMapTracking(unittest.TestCase):

    def test_diameter_prior_rejects_impossible_size(self):
        m = BucketMap()
        self.assertEqual(m.update(0, 0, 0.50, 0.0), 'rejected_diameter')
        self.assertEqual(m.update(0, 0, 0.02, 0.0), 'rejected_diameter')
        self.assertEqual(len(m.tracks), 0)

    def test_association_and_ema_converge(self):
        m = BucketMap()
        t = feed(m, [(5.0, 1.0)] * 8, jitter=0.0)
        self.assertEqual(len(m.tracks), 1)
        tr = m.tracks[0]
        self.assertEqual(tr.confirms, 8)
        self.assertAlmostEqual(tr.x, 5.0, places=9)
        self.assertLessEqual(t, 8 * 0.05)

    def test_far_detection_creates_second_track(self):
        m = BucketMap()
        feed(m, [(5.0, 1.0), (5.0, 1.0), (9.0, 3.0), (9.0, 3.0)])
        self.assertEqual(len(m.tracks), 2)

    def test_confirmation_needs_min_frames_and_low_jitter(self):
        m = BucketMap()
        feed(m, [(5.0, 1.0)] * 4)                       # 帧数不足
        self.assertEqual(len(m.confirmed(t=1.0)), 0)
        m2 = BucketMap()
        feed(m2, [(5.0, 1.0)] * 6, jitter=0.0)          # 帧足 + 不抖
        self.assertEqual(len(m2.confirmed(t=1.0)), 1)

    def test_high_jitter_blocks_confirmation(self):
        m = BucketMap()
        # 关联门 0.5 内大幅跳动：位置 std 远超 0.15
        feed(m, [(5.0, 1.0), (5.3, 1.3), (4.7, 0.7), (5.3, 0.7),
                 (4.7, 1.3), (5.0, 1.0)])
        self.assertEqual(len(m.confirmed(t=1.0)), 0)


class TestIndependenceGate(unittest.TestCase):

    def test_close_tracks_merged_into_one(self):
        """HIT 区赛'两瓶投一点'防线：间距 <0.20m 的两个确认航迹必须合并。"""
        m = BucketMap()
        feed(m, [(5.0, 1.0)] * 6)
        feed(m, [(5.12, 1.0)] * 6)     # 间距 0.12m < 0.20
        m.enforce_independence(t=1.0)
        confirmed = m.confirmed(t=1.0)
        self.assertEqual(len(confirmed), 1)
        self.assertEqual(confirmed[0].confirms, 12)  # 保留确认多者（并集）

    def test_same_size_diffonly_also_merged(self):
        """直径差 <0.025m：即使间距 0.4m（各自独立成迹）也判同一筒。"""
        m = BucketMap()
        feed(m, [(5.0, 1.0)] * 6, diameter=0.200)
        feed(m, [(5.4, 1.0)] * 6, diameter=0.208)
        m.enforce_independence(t=1.0)
        self.assertEqual(len(m.confirmed(t=1.0)), 1)

    def test_distinct_buckets_survive(self):
        m = BucketMap()
        feed(m, [(5.0, 1.0)] * 6, diameter=0.15)
        feed(m, [(5.6, 1.0)] * 6, diameter=0.25)   # 0.6m + 直径差 0.10
        m.enforce_independence(t=1.0)
        self.assertEqual(len(m.confirmed(t=1.0)), 2)


class TestLocking(unittest.TestCase):

    def make_three(self):
        m = BucketMap()
        feed(m, [(5.0, 1.0)] * 6, diameter=0.15)   # 15cm
        feed(m, [(7.0, 1.0)] * 6, diameter=0.25)   # 25cm
        feed(m, [(9.0, 1.0)] * 6, diameter=0.20)   # 20cm
        return m

    def test_rank_must_be_stable_before_lock(self):
        m = self.make_three()          # 检测止于 t≈0.25s，新鲜窗 1.0s 内完成判定
        r1 = m.try_lock(t=0.3)
        self.assertFalse(r1.ok)                    # 首次：签名刚记录
        r2 = m.try_lock(t=0.6)
        self.assertFalse(r2.ok)                    # 稳定 0.3s < 0.8s
        r3 = m.try_lock(t=1.1)
        self.assertTrue(r3.ok)                     # 0.8s 稳定 → 放行

    def test_needs_three_classified_in_normal_mode(self):
        m = BucketMap()
        feed(m, [(5.0, 1.0)] * 6, diameter=0.15)
        feed(m, [(7.0, 1.0)] * 6, diameter=0.25)
        m.try_lock(t=0.0)
        r = m.try_lock(t=1.0)
        self.assertFalse(r.ok)
        self.assertIn('classified', r.reason)

    def test_aggressive_order_small_first(self):
        m = self.make_three()
        m.try_lock(t=0.0)
        r = m.try_lock(t=1.0)
        classes = [tg.diameter_class for tg in r.targets]
        self.assertEqual(classes, [0, 1, 2])       # 15 → 20 → 25

    def test_unknown_excluded_in_normal_mode(self):
        m = self.make_three()
        feed(m, [(11.0, 1.0)] * 6, diameter=0.18)  # 在先验内但对不上号
        m.try_lock(t=0.0)
        r = m.try_lock(t=1.0)
        self.assertTrue(r.ok)
        self.assertEqual(len(r.targets), 3)        # 未知被剔除

    def test_blacklisted_track_not_relocked(self):
        m = self.make_three()
        m.try_lock(t=0.3)                          # 记录签名（3 筒）
        m.blacklist(5.0, 1.0, t=0.35)              # 已投 15cm 筒（位于 5.0,1.0）
        # SEARCH 检测流持续：剩下两筒继续可见
        feed(m, [(7.0, 1.0)] * 6, t0=1.3, diameter=0.25)
        feed(m, [(9.0, 1.0)] * 6, t0=1.6, diameter=0.20)
        r = m.try_lock(t=1.9, degraded=True)       # 降级模式（2 分类筒）重锁
        self.assertFalse(r.ok)                     # 签名变更 → 重新等稳定窗
        feed(m, [(7.0, 1.0)] * 6, t0=2.0, diameter=0.25)
        feed(m, [(9.0, 1.0)] * 6, t0=2.3, diameter=0.20)
        r = m.try_lock(t=2.7, degraded=True)       # 稳定 0.8s → 放行
        self.assertTrue(r.ok)
        # 15cm 筒不得再出现在锁定集
        self.assertFalse(any(math.hypot(tg.frozen_x - 5.0, tg.frozen_y - 1.0) < 0.1
                             for tg in r.targets))
        self.assertEqual([tg.diameter_class for tg in r.targets], [1, 2])


class TestTargetTracker(unittest.TestCase):

    def mk(self):
        tg = FrozenTarget(tid=1, frozen_x=5.0, frozen_y=1.0, frozen_diameter=0.20,
                          diameter_class=1, working_x=5.0, working_y=1.0,
                          last_vision_t=10.0)
        return tg, TargetTracker(tg, TrackParams(fresh_window_s=0.5,
                                                 reacquire_window_s=2.0))

    def test_fresh_and_lost(self):
        tg, tr = self.mk()
        self.assertEqual(tr.assess(10.2), 'ok')
        self.assertEqual(tr.assess(10.8), 'reacquiring')   # 丢 0.8s，窗口内
        self.assertAlmostEqual(tr.vision_age(10.8), 0.8, places=6)

    def test_reacquire_only_once_then_abandon(self):
        tg, tr = self.mk()
        tr.assess(10.8)                       # 进入丢视觉得记录 lost_since
        tr.on_reacquired()                    # 用掉唯一一次机会
        self.assertEqual(tr.assess(11.0), 'abandon')

    def test_reacquire_window_timeout_abandons(self):
        tg, tr = self.mk()
        tr.assess(10.8)                       # lost_since = 10.8
        self.assertEqual(tr.assess(13.0), 'abandon')   # >2s 没等到 → 弃桶

    def test_update_restores_vision(self):
        tg, tr = self.mk()
        tr.assess(10.8)
        ok = tr.update([(5.02, 0.99, 0.20)], t=11.0)
        self.assertTrue(ok)
        self.assertEqual(tr.assess(11.1), 'ok')

    def test_frozen_reference_never_rewritten(self):
        """HIT 铁律：重捕获只改活动估计，冻结参考永不动。"""
        tg, tr = self.mk()
        tr.update([(5.5, 1.2, 0.20)], t=10.1)
        tr.update([(5.6, 1.3, 0.20)], t=10.15)
        self.assertAlmostEqual(tg.frozen_x, 5.0)
        self.assertAlmostEqual(tg.frozen_y, 1.0)
        self.assertLess(math.hypot(tg.working_x - 5.0, tg.working_y - 1.0), 0.2)

    def test_update_rejects_wrong_diameter(self):
        tg, tr = self.mk()
        self.assertFalse(tr.update([(5.01, 1.0, 0.32)], t=10.1))


class TestReleaseGate(unittest.TestCase):

    def sample(self, t, horiz=0.03, vert=0.02, hs=0.03, vs=0.01,
               tilt=1.0, yaw=1.0, age=0.1, vx=0.0, vy=0.0):
        return GateSample(horiz_err_m=horiz, vert_err_m=vert, hspeed_m_s=hs,
                          vspeed_m_s=vs, tilt_deg=tilt, yaw_err_deg=yaw,
                          target_age_s=age, vx_m_s=vx, vy_m_s=vy, t=t)

    def feed_ok(self, gate, t0, dt=0.1, n=30, **kw):
        """连续喂合格样本，返回 (结果列表, 末时刻)。"""
        out = []
        t = t0
        for _ in range(n):
            out.append(gate.feed(self.sample(t, **kw)))
            t += dt
        return out, t - dt

    def test_fire_after_stability_plus_hold(self):
        g = ReleaseGate(GateParams())
        res, _ = self.feed_ok(g, 0.0)
        states = [r for r, _ in res]
        self.assertEqual(states.count('fire'), 1)
        self.assertEqual(states[-1], 'fired')          # 幂等
        # 0.8 + 1.5 = 2.3s：第 24 拍(2.3s)才 fire
        self.assertEqual(states.index('fire'), 23)

    def test_violation_resets_timers(self):
        g = ReleaseGate(GateParams())
        self.feed_ok(g, 0.0, n=10)                     # 稳 1.0s
        g.feed(self.sample(1.05, horiz=0.5))           # 违约
        res, _ = self.feed_ok(g, 1.1, n=10)            # 重新计 1.0s，还不够 2.3s
        self.assertFalse(any(s == 'fire' for s, _ in res))

    def test_stale_target_never_fires(self):
        """反盲投核心：目标估计超龄 → 永远 idle 直到超时弃桶。"""
        g = ReleaseGate(GateParams())
        res, _ = self.feed_ok(g, 0.0, n=80, age=0.9)   # 8s 全部超龄
        states = [s for s, _ in res]
        self.assertNotIn('fire', states)
        abort_entry = [r for r in res if r[0] == 'abort'][0]
        self.assertIn('target_stale', abort_entry[1])
        self.assertEqual(states[-1], 'abort')          # 终态 abort

    def test_gate_timeout_aborts(self):
        g = ReleaseGate(GateParams())
        self.feed_ok(g, 0.0, n=10)
        g.feed(self.sample(1.1, horiz=0.5))            # 打断
        res, _ = self.feed_ok(g, 1.2, n=60, horiz=0.5) # 一直不合格到超时
        self.assertEqual(res[-1][0], 'abort')

    def test_non_finite_rejected(self):
        g = ReleaseGate(GateParams())
        bad = self.sample(0.0)
        bad.horiz_err_m = float('nan')
        state, reason = g.feed(bad)
        self.assertEqual(state, 'idle')
        self.assertEqual(reason, 'non_finite')

    def test_lead_excess_sanity(self):
        g = ReleaseGate(GateParams())
        state, reason = g.feed(self.sample(0.0, hs=0.0, vx=1.0, vy=0.0))
        # v=1.0 → lead=0.6m > 0.15 上限（即便 hspeed 字段被误配成很大）
        self.assertEqual(state, 'idle')
        self.assertEqual(reason, 'lead_excess')


class TestSequencerAndAim(unittest.TestCase):

    def test_single_fire_guaranteed(self):
        s = DropSequencer()
        self.assertTrue(s.fire(0.0))
        self.assertFalse(s.fire(0.1))                  # 双发保护
        self.assertFalse(s.fire(0.2))
        self.assertIsNone(s.tick(0.5))
        self.assertEqual(s.tick(0.7), 'stow')          # 0.7s 后回仓，恰好一次
        self.assertIsNone(s.tick(0.8))

    def test_ballistic_lead(self):
        lx, ly = ballistic_lead(0.0, 0.0, 1.8)
        self.assertAlmostEqual(math.hypot(lx, ly), 0.0)
        lx, ly = ballistic_lead(0.08, 0.0, 1.8)        # 门限速度 × 落地 0.606s
        self.assertAlmostEqual(lx, 0.08 * math.sqrt(2 * 1.8 / 9.81), places=9)

    def test_aim_composes_offset_rotation_and_lead(self):
        """yaw=0（机头指向世界 +x）：机体系前向偏置应转到世界 +x。
        约定与 mission_node 一致：yaw 自 +x 起 CCW，body→local 为 c/s 旋转。"""
        tg = FrozenTarget(tid=0, frozen_x=10.0, frozen_y=0.0, frozen_diameter=0.2,
                          diameter_class=1, working_x=10.0, working_y=0.0,
                          last_vision_t=0.0)
        ax, ay, lx, ly = aim_point(tg, (0.05, 0.0), 0.0, 0.0, 0.0, GateParams())
        self.assertAlmostEqual(ax, 10.05, places=9)    # 前向偏置 → +x
        self.assertAlmostEqual(ay, 0.0, places=9)

    def test_aim_lead_clamped(self):
        tg = FrozenTarget(tid=0, frozen_x=0.0, frozen_y=0.0, frozen_diameter=0.2,
                          diameter_class=1, working_x=0.0, working_y=0.0,
                          last_vision_t=0.0)
        ax, ay, lx, ly = aim_point(tg, (0.0, 0.0), 0.0, 5.0, 0.0, GateParams())
        self.assertAlmostEqual(math.hypot(lx, ly), 0.15)   # 钳到 sanity 上限


if __name__ == '__main__':
    unittest.main()
