// =============================================================================
// test_odom_interp.cpp —— odom_interp.hpp 的单元测试
//
// 编译运行（Ubuntu/任意有 g++ 的环境；本机 Windows 可用 python -m ziglang c++）：
//   g++ -std=c++17 -I ../include test_odom_interp.cpp -o test_odom_interp && ./test_odom_interp
// 全部通过打印 PASS 并返回 0；任一失败打印行号并返回 1（可接 CI）。
//
// 覆盖 Re0 十二项回归清单的时间域三项：②时间回跳（超老夹逼/拒绝）、
// ③插值等号边界（端点/中间样本恰等）、④大间隔（严格大于才拒，恰等放行）。
// Python 镜像 = tools/test_odom_interp.py，两边一一对应。
// =============================================================================

#include <cmath>
#include <cstdio>
#include <deque>

#include "cuadc_mission/odom_interp.hpp"

using cuadc_odom::InterpParams;
using cuadc_odom::InterpStatus;
using cuadc_odom::Sample;

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

static std::deque<Sample> hist(std::initializer_list<Sample> pts)
{
  return std::deque<Sample>(pts);
}

int main()
{
  const InterpParams p;   // 0.05 / 1.5 / 0.2

  {   // 空历史
    const auto r = cuadc_odom::interpolate({}, 1.0, p);
    CHECK(r.status == InterpStatus::kEmpty);
  }
  {   // 恰等未来戳容差 → 放行返回 newest（容差取 0.0625=2^-4 保证 FP 精确等号）
    const InterpParams p2{0.0625, 1.5, 0.2};
    const auto h = hist({{0.0, 0, 0, 0, 0}, {1.0, 1, 0, 0, 0}});
    const auto r = cuadc_odom::interpolate(h, 1.0625, p2);
    CHECK(r.status == InterpStatus::kOk);
    CHECK_NEAR(r.out.t, 1.0, 1e-12);
    CHECK_NEAR(r.out.x, 1.0, 1e-12);
    CHECK(cuadc_odom::interpolate(h, 1.0626, p2).status == InterpStatus::kFutureStamp);
  }
  {   // 未来戳超容差拒绝
    const auto h = hist({{0.0, 0, 0, 0, 0}, {1.0, 1, 0, 0, 0}});
    CHECK(cuadc_odom::interpolate(h, 1.051, p).status == InterpStatus::kFutureStamp);
  }
  {   // 恰等 max_delay（-1.5）→ 夹逼返回 oldest（超老但在延迟窗内不丢弃）
    const auto h = hist({{0.0, 5, 5, 2, 0.3}, {1.0, 6, 5, 2, 0.3}});
    const auto r = cuadc_odom::interpolate(h, -0.5, p);
    CHECK(r.status == InterpStatus::kOk);
    CHECK_NEAR(r.out.x, 5.0, 1e-12);
    CHECK_NEAR(r.out.yaw, 0.3, 1e-12);
  }
  {   // 超老拒绝
    const auto h = hist({{0.0, 5, 5, 2, 0.3}, {1.0, 6, 5, 2, 0.3}});
    CHECK(cuadc_odom::interpolate(h, -0.51, p).status == InterpStatus::kTooOld);
  }
  {   // t 恰等于中间样本戳 → 原样返回该样本（样本间隔 0.1s 须 ≤ max_gap）
    const auto h = hist({{0.0, 0, 0, 0, 0}, {0.1, 1, 1, 1, 0.1}, {0.2, 2, 2, 2, 0.2}});
    const auto r = cuadc_odom::interpolate(h, 0.1, p);
    CHECK(r.status == InterpStatus::kOk);
    CHECK_NEAR(r.out.t, 0.1, 1e-12);
    CHECK_NEAR(r.out.x, 1.0, 1e-12);
    CHECK_NEAR(r.out.y, 1.0, 1e-12);
    CHECK_NEAR(r.out.yaw, 0.1, 1e-12);
  }
  {   // 中点线性插值
    const auto h = hist({{0.0, 0, 0, 2, 0.0}, {0.2, 1, 2, 2.2, 0.5}});
    const auto r = cuadc_odom::interpolate(h, 0.1, p);
    CHECK(r.status == InterpStatus::kOk);
    CHECK_NEAR(r.out.x, 0.5, 1e-12);
    CHECK_NEAR(r.out.y, 1.0, 1e-12);
    CHECK_NEAR(r.out.z, 2.1, 1e-12);
    CHECK_NEAR(r.out.yaw, 0.25, 1e-12);
  }
  {   // yaw 跨 ±π 走最短角差：a=+3.1, b=-3.1 → 中点 ≈ π
    const auto h = hist({{0.0, 0, 0, 0, 3.1}, {0.2, 0, 0, 0, -3.1}});
    const auto r = cuadc_odom::interpolate(h, 0.1, p);
    CHECK(r.status == InterpStatus::kOk);
    CHECK_NEAR(r.out.yaw, 3.14159265358979323846, 1e-6);
  }
  {   // 大间隔：恰 0.2s 放行；0.21s 拒绝（严格大于才拒）
    const auto h = hist({{0.0, 0, 0, 0, 0}, {0.2, 1, 0, 0, 0}});
    const auto r = cuadc_odom::interpolate(h, 0.1, p);
    CHECK(r.status == InterpStatus::kOk);
    CHECK_NEAR(r.out.x, 0.5, 1e-12);
    const auto h2 = hist({{0.0, 0, 0, 0, 0}, {0.21, 1, 0, 0, 0}});
    CHECK(cuadc_odom::interpolate(h2, 0.1, p).status == InterpStatus::kGapBreak);
  }
  {   // 时间回跳（t 早于 oldest 但在延迟窗内）→ 夹逼 oldest
    const auto h = hist({{0.0, 1, 0, 0, 0}, {0.1, 1.1, 0, 0, 0}});
    const auto r = cuadc_odom::interpolate(h, -0.02, p);
    CHECK(r.status == InterpStatus::kOk);
    CHECK_NEAR(r.out.x, 1.0, 1e-12);
  }

  std::printf("%d checks, %d failures -> %s\n", g_checks, g_failures,
    g_failures == 0 ? "PASS" : "FAIL");
  return g_failures == 0 ? 0 : 1;
}
