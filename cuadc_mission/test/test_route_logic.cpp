// =============================================================================
// test_route_logic.cpp —— route_logic.hpp 的单元测试
//
// 编译运行（Ubuntu/任意有 g++ 的环境；本机 Windows 可用 python -m ziglang c++）：
//   g++ -std=c++17 -I ../include test_route_logic.cpp -o test_route_logic && ./test_route_logic
// 全部通过打印 PASS 并返回 0；任一失败打印行号并返回 1（可接 CI）。
//
// 三条防线：
//   1. 罗盘→ENU 换算与场地旋转的回归锚点（2026-09-12"横着搜"修复的守门用例：
//      出生 yaw 与场地前进方向错 90° 的那类 bug，在这里必红）；
//   2. 蛇形航线形态：航点数/带位均布/奇偶交替/换轴语义；
//   3. 覆盖判据：区内任一点到最近带的横向距离 ≤ 下视相机横向半覆盖
//      （1.055×高度 @hfov1.5rad、848×480），杜绝搜索盲区。
// =============================================================================

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "cuadc_mission/route_logic.hpp"

using cuadc_route::kPi;
using cuadc_route::field_to_local;
using cuadc_route::heading_enu_rad_from_compass_deg;

using cuadc_route::FieldPoint;
using cuadc_route::Point3;

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

/// 归一化到 (-pi, pi] 后比较弧度
static void check_angle_near(double a, double b, double tol, int line)
{
  ++g_checks;
  double d = std::fmod(a - b, 2.0 * cuadc_route::kPi);
  if (d > cuadc_route::kPi) {
    d -= 2.0 * cuadc_route::kPi;
  }
  if (d < -cuadc_route::kPi) {
    d += 2.0 * cuadc_route::kPi;
  }
  if (std::fabs(d) > tol) {
    ++g_failures;
    std::printf("FAIL test_route_logic.cpp:%d  angle |%.6f - %.6f| > %g\n",
      line, a, b, tol);
  }
}

#define CHECK_ANGLE_NEAR(a, b, tol) check_angle_near((a), (b), (tol), __LINE__)

// ---------------------------------------------------------------------------
// 防线 1：罗盘→ENU 换算 + 场地旋转（"横着搜"回归锚点）
// ---------------------------------------------------------------------------
static void test_heading_conversion()
{
  // 罗盘 0/90/180/270（N/E/S/W）→ ENU π/2/0/−π/2/π
  CHECK_ANGLE_NEAR(heading_enu_rad_from_compass_deg(0.0), cuadc_route::kPi / 2, 1e-12);
  CHECK_ANGLE_NEAR(heading_enu_rad_from_compass_deg(90.0), 0.0, 1e-12);
  CHECK_ANGLE_NEAR(heading_enu_rad_from_compass_deg(180.0), -cuadc_route::kPi / 2, 1e-12);
  CHECK_ANGLE_NEAR(heading_enu_rad_from_compass_deg(270.0), cuadc_route::kPi, 1e-12);
  CHECK_ANGLE_NEAR(heading_enu_rad_from_compass_deg(359.0), cuadc_route::kPi / 2 + 1.0 * cuadc_route::kPi / 180.0, 1e-9);
}

static void test_field_to_local_anchors()
{
  const double tol = 1e-12;
  // 4 个罗盘锚点：机头方向 field(1,0) 必须落在地理正确的 ENU 方向上
  struct Case { double compass; double ex, ey; };
  const Case cases[] = {
    {0.0, 0.0, 1.0},     // 朝北：前=北(+Y)
    {90.0, 1.0, 0.0},    // 朝东：前=东(+X)
    {180.0, 0.0, -1.0},  // 朝南：前=南(−Y)
    {270.0, -1.0, 0.0},  // 朝西：前=西(−X)
  };
  for (const auto & c : cases) {
    const double enu = heading_enu_rad_from_compass_deg(c.compass);
    const Point3 fwd = field_to_local(enu, 0, 0, 0, 1, 0, 0);
    CHECK_NEAR(fwd.x, c.ex, tol);
    CHECK_NEAR(fwd.y, c.ey, tol);
  }
  // 朝东时"左"= 北；朝北时"左"= 西
  const Point3 left_east = field_to_local(0.0, 0, 0, 0, 0, 1, 0);
  CHECK_NEAR(left_east.x, 0.0, tol);
  CHECK_NEAR(left_east.y, 1.0, tol);
  const Point3 left_north = field_to_local(cuadc_route::kPi / 2, 0, 0, 0, 0, 1, 0);
  CHECK_NEAR(left_north.x, -1.0, tol);
  CHECK_NEAR(left_north.y, 0.0, tol);
  // 平移与 z 直通
  const Point3 shifted = field_to_local(0.0, 10, 20, 1, 1, 2, 0.5);
  CHECK_NEAR(shifted.x, 11.0, tol);
  CHECK_NEAR(shifted.y, 22.0, tol);
  CHECK_NEAR(shifted.z, 1.5, tol);
  // 本修复的直接场景：场地沿世界 +X（桶在东），出生机头必须朝东（compass≈90），
  // field x=6 的航点必须落在世界 +X——错 90° 时它落在 +Y（"横着搜"）
  const double yaw_east = heading_enu_rad_from_compass_deg(90.0);
  const Point3 p = field_to_local(yaw_east, 0, 0, 0, 6, -2, 2);
  CHECK_NEAR(p.x, 6.0, tol);
  CHECK_NEAR(p.y, -2.0, tol);
  const double yaw_wrong = heading_enu_rad_from_compass_deg(0.0);   // 出生朝北=bug 状态
  const Point3 bad = field_to_local(yaw_wrong, 0, 0, 0, 6, -2, 2);
  CHECK(std::fabs(bad.x - 6.0) > 1.0);   // 必然不在场地 +X 上（守护断言）
}

// ---------------------------------------------------------------------------
// 防线 2：蛇形航线形态
// ---------------------------------------------------------------------------
static void test_serpentine_along_x()
{
  // 搜索区现状：x 27.5~32.5（带长 5m），y ±4（带宽 8m），6 带
  const auto r = cuadc_route::build_serpentine(27.5, 32.5, -4.0, 4.0, 6, true);
  CHECK(r.size() == 12U);
  const double tol = 1e-12;
  // 带 0（y=-4，正向）：x0→x1
  CHECK_NEAR(r[0].x, 27.5, tol);  CHECK_NEAR(r[0].y, -4.0, tol);
  CHECK_NEAR(r[1].x, 32.5, tol);  CHECK_NEAR(r[1].y, -4.0, tol);
  // 带 1（y=-2.4，反向）：x1→x0
  CHECK_NEAR(r[2].x, 32.5, tol);  CHECK_NEAR(r[2].y, -2.4, tol);
  CHECK_NEAR(r[3].x, 27.5, tol);  CHECK_NEAR(r[3].y, -2.4, tol);
  // 带 5（y=4，奇数带反向）：x1→x0
  CHECK_NEAR(r[10].x, 32.5, tol); CHECK_NEAR(r[10].y, 4.0, tol);
  CHECK_NEAR(r[11].x, 27.5, tol); CHECK_NEAR(r[11].y, 4.0, tol);
  // 带位均布：y = -4 + 8·lane/5
  for (int lane = 0; lane < 6; ++lane) {
    CHECK_NEAR(r[2 * lane].y, -4.0 + 8.0 * lane / 5.0, 1e-9);
  }
  // 单带居中
  const auto one = cuadc_route::build_serpentine(0, 10, -2, 2, 1, true);
  CHECK(one.size() == 2U);
  CHECK_NEAR(one[0].y, 0.0, tol);
}

static void test_serpentine_along_y()
{
  // 侦察区（09-12 重排）：带位沿 x 52.5~57.5 均布，带沿 y ±4（长轴 8m）扫
  const auto r = cuadc_route::build_serpentine(52.5, 57.5, -4.0, 4.0, 3, false);
  CHECK(r.size() == 6U);
  const double tol = 1e-12;
  // 带 0（x=52.5，正向）：y0→y1
  CHECK_NEAR(r[0].x, 52.5, tol); CHECK_NEAR(r[0].y, -4.0, tol);
  CHECK_NEAR(r[1].x, 52.5, tol); CHECK_NEAR(r[1].y, 4.0, tol);
  // 带 1（x=55，反向）：y1→y0
  CHECK_NEAR(r[2].x, 55.0, tol); CHECK_NEAR(r[2].y, 4.0, tol);
  CHECK_NEAR(r[3].x, 55.0, tol); CHECK_NEAR(r[3].y, -4.0, tol);
  // 带 2（x=57.5，正向）
  CHECK_NEAR(r[4].x, 57.5, tol); CHECK_NEAR(r[4].y, -4.0, tol);
  CHECK_NEAR(r[5].x, 57.5, tol); CHECK_NEAR(r[5].y, 4.0, tol);
}

// ---------------------------------------------------------------------------
// 防线 3：相机覆盖判据（区内无搜索盲区）
// ---------------------------------------------------------------------------
static void test_camera_coverage()
{
  using cuadc_route::nadir_across_half_width;
  const double across2 = nadir_across_half_width(2.0, 1.5, 848.0, 480.0);
  CHECK_NEAR(across2, 2.0 * std::tan(0.75) * 480.0 / 848.0, 1e-9);   // ≈1.0546
  CHECK(across2 > 0.80 && across2 < 1.20);                           // 量级护栏
  const double across4 = nadir_across_half_width(4.0, 1.5, 848.0, 480.0);
  CHECK_NEAR(across4, 2.0 * across2, 1e-9);                          // 线性于高度

  // 搜索区 6 带覆盖：网格采样区内点，到最近带的横向距离 ≤ 半覆盖
  const auto search = cuadc_route::build_serpentine(27.5, 32.5, -4.0, 4.0, 6, true);
  double worst = 0.0;
  for (double x = 27.5; x <= 32.5 + 1e-9; x += 0.5) {  // 沿 x 带横向距离只与 y 有关
    for (double y = -4.0; y <= 4.0 + 1e-9; y += 0.1) {
      double best = 1e9;
      for (std::size_t i = 0; i < search.size(); i += 2) {  // 每带首点即带位
        best = std::min(best, std::fabs(y - search[i].y));
      }
      worst = std::max(worst, best);
    }
  }
  CHECK(worst <= across2 + 1e-9);          // 6 带无盲区（0.8 ≤ 1.05）
  CHECK(worst > 0.0);                      // 采样确实发生了
  CHECK_NEAR(cuadc_route::max_cross_track_distance(8.0, 6), 0.8, 1e-12);

  // 侦察区 3 带（沿 y，横向=x）：跨度 5m，最远 1.25m ≤ 4m 高半覆盖 2.11m
  const auto recon = cuadc_route::build_serpentine(52.5, 57.5, -4.0, 4.0, 3, false);
  double worst_r = 0.0;
  for (double x = 52.5; x <= 57.5 + 1e-9; x += 0.1) {
    double best = 1e9;
    for (std::size_t i = 0; i < recon.size(); i += 2) {
      best = std::min(best, std::fabs(x - recon[i].x));
    }
    worst_r = std::max(worst_r, best);
  }
  CHECK(worst_r <= across4 + 1e-9);
  CHECK_NEAR(cuadc_route::max_cross_track_distance(5.0, 3), 1.25, 1e-12);
  CHECK_NEAR(cuadc_route::max_cross_track_distance(8.0, 1), 4.0, 1e-12);
}

int main()
{
  test_heading_conversion();
  test_field_to_local_anchors();
  test_serpentine_along_x();
  test_serpentine_along_y();
  test_camera_coverage();

  std::printf("%d checks, %d failures\n", g_checks, g_failures);
  if (g_failures == 0) {
    std::printf("PASS\n");
    return 0;
  }
  return 1;
}
