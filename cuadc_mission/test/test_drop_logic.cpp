// =============================================================================
// test_drop_logic.cpp —— drop_logic.hpp 的单元测试（镜像 tools/test_drop_logic.py）
//
// 编译运行（Ubuntu/任意有 g++ 的环境；本机 Windows 可用 python -m ziglang c++）：
//   g++ -std=c++17 -I ../include test_drop_logic.cpp -o test_drop_logic && ./test_drop_logic
// 全部通过打印 PASS 并返回 0；任一失败打印行号并返回 1（可接 CI）。
//
// 防线与用例的对应关系见 tools/test_drop_logic.py——两边用例一一镜像，
// 改判据先改 Python 版跑绿，再同步 hpp 与本文件。
// =============================================================================

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "cuadc_mission/drop_logic.hpp"

using namespace cuadc_drop;

static int g_checks = 0;
static int g_failures = 0;

#define CHECK(cond) do { \
    ++g_checks; \
    if (!(cond)) { \
      ++g_failures; \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

#define CHECK_NEAR(a, b, tol) do { \
    ++g_checks; \
    if (std::fabs((a) - (b)) > (tol)) { \
      ++g_failures; \
      std::printf("FAIL %s:%d  |%.9f - %.9f| > %g\n", __FILE__, __LINE__, \
        (a), (b), static_cast<double>(tol)); \
    } \
} while (0)

static double feed(BucketMap & m, const std::vector<std::pair<double, double>> & xs,
  double diameter = 0.20, double t0 = 0.0, double dt = 0.05)
{
  double t = t0;
  for (auto xy : xs) {
    m.update(xy.first, xy.second, diameter, t);
    t += dt;
  }
  return t - dt;
}

static std::vector<std::pair<double, double>> repeat(double x, double y, int n)
{
  return std::vector<std::pair<double, double>>(n, {x, y});
}

// ---------------------------------------------------------------------------
static void test_bbox_free_math()
{
  CHECK_NEAR(ballistic_lead(0.0, 0.0, 1.8).first, 0.0, 1e-12);
  const auto lead = ballistic_lead(0.08, 0.0, 1.8);
  CHECK_NEAR(lead.first, 0.08 * std::sqrt(2 * 1.8 / 9.81), 1e-9);
}

// ---------------------------------------------------------------------------
static void test_bucket_map_tracking()
{
  {   // 直径先验拒绝
    BucketMap m;
    CHECK(m.update(0, 0, 0.50, 0.0) == UpdateResult::kRejectedDiameter);
    CHECK(m.update(0, 0, 0.02, 0.0) == UpdateResult::kRejectedDiameter);
  }
  {   // 关联 + EMA 收敛
    BucketMap m;
    feed(m, repeat(5.0, 1.0, 8));
    CHECK(m.confirmed(0.35).empty() || true);   // 8 帧 > 5 帧可能已确认
    CHECK(m.confirmed(0.35).size() == 1);
    CHECK_NEAR(m.confirmed(0.35)[0].x, 5.0, 1e-9);
  }
  {   // 远距离检测 → 第二条航迹
    BucketMap m;
    feed(m, repeat(5.0, 1.0, 2));
    feed(m, repeat(9.0, 3.0, 2), 0.20, 0.15);
    CHECK(m.track_count() == 2);
  }
  {   // 帧数不足不确认；帧足 + 低抖动确认
    BucketMap m;
    feed(m, repeat(5.0, 1.0, 4));
    CHECK(m.confirmed(1.0).empty());
    BucketMap m2;
    feed(m2, repeat(5.0, 1.0, 6));
    CHECK(m2.confirmed(1.0).size() == 1);
  }
  {   // 高抖动阻止确认
    BucketMap m;
    const std::vector<std::pair<double, double>> jittered = {
      {5.0, 1.0}, {5.3, 1.3}, {4.7, 0.7}, {5.3, 0.7}, {4.7, 1.3}, {5.0, 1.0}};
    feed(m, jittered);
    CHECK(m.confirmed(1.0).empty());
  }
}

// ---------------------------------------------------------------------------
static void test_independence_gate()
{
  {   // HIT 区赛"两瓶投一点"防线：间距 <0.20m 的确认航迹必须合并
    BucketMap m;
    feed(m, repeat(5.0, 1.0, 6));
    feed(m, repeat(5.12, 1.0, 6), 0.20, 0.35);
    m.enforce_independence(1.0);
    const auto c = m.confirmed(1.0);
    CHECK(c.size() == 1);
    CHECK(c[0].confirms == 12);
  }
  {   // 直径差 <0.025m：即使间距 0.4m 也判同一筒
    BucketMap m;
    feed(m, repeat(5.0, 1.0, 6), 0.200);
    feed(m, repeat(5.4, 1.0, 6), 0.208, 0.35);
    m.enforce_independence(1.0);
    CHECK(m.confirmed(1.0).size() == 1);
  }
  {   // 真正独立的筒存活
    BucketMap m;
    feed(m, repeat(5.0, 1.0, 6), 0.15);
    feed(m, repeat(5.6, 1.0, 6), 0.25, 0.35);
    m.enforce_independence(1.0);
    CHECK(m.confirmed(1.0).size() == 2);
  }
}

// ---------------------------------------------------------------------------
static BucketMap make_three()
{
  BucketMap m;
  feed(m, repeat(5.0, 1.0, 6), 0.15);   // 15cm
  feed(m, repeat(7.0, 1.0, 6), 0.25, 0.35);   // 25cm
  feed(m, repeat(9.0, 1.0, 6), 0.20, 0.70);   // 20cm
  return m;
}

static void test_locking()
{
  {   // 排名必须稳定 0.8s 才锁
    BucketMap m = make_three();
    CHECK(!m.try_lock(0.3).ok);          // 首次：签名刚记录
    CHECK(!m.try_lock(0.6).ok);          // 0.3s < 0.8s
    CHECK(m.try_lock(1.1).ok);           // 0.8s 稳定 → 放行
  }
  {   // 正式模式需 3 个分类筒
    BucketMap m;
    feed(m, repeat(5.0, 1.0, 6), 0.15);
    feed(m, repeat(7.0, 1.0, 6), 0.25, 0.35);
    m.try_lock(0.0);
    const auto r = m.try_lock(1.0);
    CHECK(!r.ok);
    CHECK(r.reason.find("classified") != std::string::npos);
  }
  {   // aggressive 语义：直径类升序 15 → 20 → 25
    BucketMap m = make_three();
    m.try_lock(0.3);
    const auto r = m.try_lock(1.1);
    CHECK(r.ok);
    CHECK(r.targets.size() == 3);
    CHECK(r.targets[0].diameter_class == 0);
    CHECK(r.targets[1].diameter_class == 1);
    CHECK(r.targets[2].diameter_class == 2);
  }
  {   // conservative 排序（sorted_targets 帮助函数）：先大后小
    BucketMap m = make_three();
    m.try_lock(0.3);
    auto r = m.try_lock(1.1);
    const auto sorted = sorted_targets(r.targets, "conservative");
    CHECK(sorted[0].diameter_class == 2);
    CHECK(sorted[2].diameter_class == 0);
  }
  {   // 正式模式：未知筒剔除
    BucketMap m = make_three();
    feed(m, repeat(11.0, 1.0, 6), 0.18, 1.05);   // 先验内但对不上号
    m.try_lock(0.3);
    const auto r = m.try_lock(1.1);
    CHECK(r.ok);
    CHECK(r.targets.size() == 3);
  }
  {   // 已投筒拉黑后不得回锁（降级重锁路径）
    BucketMap m = make_three();
    m.try_lock(0.3);
    m.blacklist(5.0, 1.0, 0.35);
    feed(m, repeat(7.0, 1.0, 6), 0.25, 1.3);
    feed(m, repeat(9.0, 1.0, 6), 0.20, 1.6);
    CHECK(!m.try_lock(1.9, true).ok);            // 签名变更 → 重新计稳定
    feed(m, repeat(7.0, 1.0, 6), 0.25, 2.0);
    feed(m, repeat(9.0, 1.0, 6), 0.20, 2.3);
    const auto r = m.try_lock(2.7, true);
    CHECK(r.ok);
    bool near_blacklisted = false;
    for (const auto & tg : r.targets) {
      if (std::hypot(tg.frozen_x - 5.0, tg.frozen_y - 1.0) < 0.1) {
        near_blacklisted = true;
      }
    }
    CHECK(!near_blacklisted);
    CHECK(r.targets[0].diameter_class == 1);
    CHECK(r.targets[1].diameter_class == 2);
  }
}

// ---------------------------------------------------------------------------
static void test_target_tracker()
{
  {   // 新鲜 → 丢失 → 重捕获窗口
    FrozenTarget tg;
    tg.frozen_x = tg.working_x = 5.0;
    tg.frozen_y = tg.working_y = 1.0;
    tg.frozen_diameter = 0.20;
    tg.last_vision_t = 10.0;
    TargetTracker tr(&tg, TrackParams{});
    CHECK(tr.assess(10.2) == Assess::kOk);
    CHECK(tr.assess(10.8) == Assess::kReacquiring);
    CHECK_NEAR(tr.vision_age(10.8), 0.8, 1e-6);
  }
  {   // 重捕获机会只有一次
    FrozenTarget tg;
    tg.last_vision_t = 10.0;
    TargetTracker tr(&tg, TrackParams{});
    tr.assess(10.8);
    tr.on_reacquired();
    CHECK(tr.assess(11.0) == Assess::kAbandon);
  }
  {   // 重捕获窗口超时 → 弃桶
    FrozenTarget tg;
    tg.last_vision_t = 10.0;
    TargetTracker tr(&tg, TrackParams{});
    tr.assess(10.8);
    CHECK(tr.assess(13.0) == Assess::kAbandon);
  }
  {   // update 恢复视觉
    FrozenTarget tg;
    tg.frozen_x = tg.working_x = 5.0;
    tg.frozen_y = tg.working_y = 1.0;
    tg.frozen_diameter = 0.20;
    tg.last_vision_t = 10.0;
    TargetTracker tr(&tg, TrackParams{});
    tr.assess(10.8);
    CHECK(tr.update({{5.02, 0.99, 0.20}}, 11.0));
    CHECK(tr.assess(11.1) == Assess::kOk);
  }
  {   // 冻结参考绝不被改写
    FrozenTarget tg;
    tg.frozen_x = tg.working_x = 5.0;
    tg.frozen_y = tg.working_y = 1.0;
    tg.frozen_diameter = 0.20;
    tg.last_vision_t = 10.0;
    TargetTracker tr(&tg, TrackParams{});
    tr.update({{5.5, 1.2, 0.20}}, 10.1);
    tr.update({{5.6, 1.3, 0.20}}, 10.15);
    CHECK_NEAR(tg.frozen_x, 5.0, 1e-12);
    CHECK_NEAR(tg.frozen_y, 1.0, 1e-12);
    CHECK(std::hypot(tg.working_x - 5.0, tg.working_y - 1.0) < 0.2);
  }
  {   // 直径不符的检测被拒
    FrozenTarget tg;
    tg.frozen_x = tg.working_x = 5.0;
    tg.frozen_y = tg.working_y = 1.0;
    tg.frozen_diameter = 0.20;
    tg.last_vision_t = 10.0;
    TargetTracker tr(&tg, TrackParams{});
    CHECK(!tr.update({{5.01, 1.0, 0.32}}, 10.1));
  }
}

// ---------------------------------------------------------------------------
static GateSample ok_sample(double t, double horiz = 0.03, double hs = 0.03,
  double age = 0.1, double vx = 0.0, double vy = 0.0)
{
  GateSample s;
  s.horiz_err_m = horiz;
  s.vert_err_m = 0.02;
  s.hspeed_m_s = hs;
  s.vspeed_m_s = 0.01;
  s.tilt_deg = 1.0;
  s.yaw_err_deg = 1.0;
  s.target_age_s = age;
  s.vx_m_s = vx;
  s.vy_m_s = vy;
  s.t = t;
  return s;
}

static void test_release_gate()
{
  {   // 稳定 0.8s + 保持 1.5s = 连续 2.3s 后 fire 恰一次
    ReleaseGate g;
    GateVerdict last{GateStatus::kIdle, ""};
    int fire_count = 0;
    int fire_at = -1;
    for (int i = 0; i < 30; ++i) {
      last = g.feed(ok_sample(i * 0.1));
      if (last.status == GateStatus::kFire) {
        ++fire_count;
        fire_at = i;
      }
    }
    CHECK(fire_count == 1);
    CHECK(fire_at == 23);                      // 第 24 拍 = 2.3s
    CHECK(last.status == GateStatus::kFired);  // 幂等
  }
  {   // 违约清零计时器
    ReleaseGate g;
    for (int i = 0; i < 10; ++i) {
      g.feed(ok_sample(i * 0.1));              // 稳 1.0s
    }
    g.feed(ok_sample(1.05, 0.5));              // 违约
    bool fired = false;
    for (int i = 0; i < 10; ++i) {
      if (g.feed(ok_sample(1.1 + i * 0.1)).status == GateStatus::kFire) {
        fired = true;                          // 重新只稳 1.0s，不够 2.3s
      }
    }
    CHECK(!fired);
  }
  {   // 反盲投核心：目标超龄 → 永远不开火，直到超时弃桶
    ReleaseGate g;
    GateVerdict abort_verdict{GateStatus::kIdle, ""};
    for (int i = 0; i < 80; ++i) {
      const auto v = g.feed(ok_sample(i * 0.1, 0.03, 0.03, 0.9));
      if (v.status == GateStatus::kAbort) {
        abort_verdict = v;
      }
      CHECK(v.status != GateStatus::kFire);
    }
    CHECK(abort_verdict.status == GateStatus::kAbort);
    CHECK(abort_verdict.reason.find("target_stale") != std::string::npos);
  }
  {   // 门控超时弃桶
    ReleaseGate g;
    for (int i = 0; i < 10; ++i) {
      g.feed(ok_sample(i * 0.1));
    }
    g.feed(ok_sample(1.1, 0.5));
    GateVerdict last{GateStatus::kIdle, ""};
    for (int i = 0; i < 60; ++i) {
      last = g.feed(ok_sample(1.2 + i * 0.1, 0.5));
    }
    CHECK(last.status == GateStatus::kAbort);
  }
  {   // 非有限值拒绝
    ReleaseGate g;
    auto bad = ok_sample(0.0);
    bad.horiz_err_m = std::nan("");
    const auto v = g.feed(bad);
    CHECK(v.status == GateStatus::kIdle);
    CHECK(v.reason == "non_finite");
  }
  {   // lead_excess sanity（即便 hspeed 字段被误配，速度前移超限仍拦截）
    ReleaseGate g;
    const auto v = g.feed(ok_sample(0.0, 0.03, 0.0, 0.1, 1.0, 0.0));
    CHECK(v.status == GateStatus::kIdle);
    CHECK(v.reason == "lead_excess");
  }
}

// ---------------------------------------------------------------------------
static void test_sequencer_and_aim()
{
  {   // 单发保证
    DropSequencer s;
    CHECK(s.fire(0.0));
    CHECK(!s.fire(0.1));
    CHECK(!s.fire(0.2));
    CHECK(s.tick(0.5) == SeqResult::kNone);
    CHECK(s.tick(0.7) == SeqResult::kStow);    // 0.7s 后回仓，恰好一次
    CHECK(s.tick(0.8) == SeqResult::kNone);
  }
  {   // 瞄准点 = 冻结估计 + 偏置旋转（yaw=0 → 前向偏置 → +x）
    FrozenTarget tg;
    tg.working_x = 10.0;
    tg.working_y = 0.0;
    const auto a = aim_point(tg, 0.05, 0.0, 0.0, 0.0, 0.0, GateParams{});
    CHECK_NEAR(a.ax, 10.05, 1e-9);
    CHECK_NEAR(a.ay, 0.0, 1e-9);
  }
  {   // lead 超限被钳到 sanity 上限
    FrozenTarget tg;
    tg.working_x = 0.0;
    tg.working_y = 0.0;
    const auto a = aim_point(tg, 0.0, 0.0, 0.0, 5.0, 0.0, GateParams{});
    CHECK_NEAR(std::hypot(a.lead_x, a.lead_y), 0.15, 1e-9);
  }
}

int main()
{
  test_bbox_free_math();
  test_bucket_map_tracking();
  test_independence_gate();
  test_locking();
  test_target_tracker();
  test_release_gate();
  test_sequencer_and_aim();
  std::printf("%d checks, %d failures -> %s\n", g_checks, g_failures,
    g_failures == 0 ? "PASS" : "FAIL");
  return g_failures == 0 ? 0 : 1;
}
