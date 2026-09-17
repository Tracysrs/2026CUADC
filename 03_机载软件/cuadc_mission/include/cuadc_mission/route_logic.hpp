// =============================================================================
// route_logic.hpp —— 航线构建与场地/ENU 坐标换算的纯函数（无 ROS 依赖，可单测）
//
// 设计依据：01_设计/总体方案与执行计划.md（SSOT）§5.1 航线形态、§10.2-1 坐标
// 系列坑；2026-09-12"横着搜"修复（出生 yaw=90° vs 场地沿 +X）的回归防线。
// 与 mission_node.cpp 的关系：build_search_route/build_recon_route/field_to_local
// 委托到本头文件；单测见 test/test_route_logic.cpp（编译方式同 test_drop_logic）。
// =============================================================================

#ifndef CUADC_MISSION__ROUTE_LOGIC_HPP_
#define CUADC_MISSION__ROUTE_LOGIC_HPP_

#include <cmath>
#include <vector>

namespace cuadc_route
{

constexpr double kPi = 3.14159265358979323846;  // M_PI 在严格模式不可用，自带常量

/// 本地 ENU 三维点（米，z 向上，相对上电原点）
struct Point3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

/// 场地系平面点（原点=起飞点，x=锁定的机头方向，y=左）
struct FieldPoint
{
  double x = 0.0;
  double y = 0.0;
};

/// 罗盘航向（度，0=N 顺时针）→ ENU 航向角（弧度，自 +X 逆时针）。
/// SSOT §10.2-1：yaw_ENU = 90° − compass，坐标系列坑之首，勿改。
inline double heading_enu_rad_from_compass_deg(double compass_deg)
{
  return (90.0 - compass_deg) * kPi / 180.0;
}

/// 场地系（x 机头，y 左）→ 本地 ENU。yaw_enu = 起飞瞬间 ENU 航向（弧度）。
/// 回归锚点：罗盘 0°（机头朝北）时 field(1,0) → ENU(0,1)（北=机头方向），
/// field(0,1)（机头左侧）→ ENU(-1,0)（西=朝北时的左）。
inline Point3 field_to_local(
  double yaw_enu, double origin_x, double origin_y, double origin_z,
  double fx, double fy, double fz)
{
  const double c = std::cos(yaw_enu);
  const double s = std::sin(yaw_enu);
  return Point3{
    origin_x + c * fx - s * fy,
    origin_y + s * fx + c * fy,
    origin_z + fz};
}

/// 弓字形蛇形航线（场地系，按飞行顺序，2×lanes 个航点，lanes≥1）。
/// along_x=true ：带沿 x 扫（x0↔x1），带位沿 y 在 [y0,y1] 均布（搜索区现状）；
/// along_x=false：带沿 y 扫（y0↔y1），带位沿 x 在 [x0,x1] 均布（2026-09-12
/// 侦察区重排：带沿区长轴 8m 扫全宽，横向间距减半，覆盖余量 0.11m→0.86m）。
/// 奇偶带交替方向；lanes==1 时带位居中。
inline std::vector<FieldPoint> build_serpentine(
  double x0, double x1, double y0, double y1, int lanes, bool along_x)
{
  std::vector<FieldPoint> route;
  if (lanes < 1) {
    return route;
  }
  for (int lane = 0; lane < lanes; ++lane) {
    const double t = lanes == 1 ?
      0.5 : static_cast<double>(lane) / static_cast<double>(lanes - 1);
    const bool forward = lane % 2 == 0;
    if (along_x) {
      const double y = y0 + (y1 - y0) * t;
      route.push_back(FieldPoint{forward ? x0 : x1, y});
      route.push_back(FieldPoint{forward ? x1 : x0, y});
    } else {
      const double x = x0 + (x1 - x0) * t;
      route.push_back(FieldPoint{x, forward ? y0 : y1});
      route.push_back(FieldPoint{x, forward ? y1 : y0});
    }
  }
  return route;
}

/// 下视（nadir）相机横向（垂直于带方向）半覆盖宽度（米）。
/// SDF horizontal_fov 张在图像宽度（848px，沿飞行方向）上，横向沿高度方向；
/// tan(vfov/2) = tan(hfov/2)·(H_px/W_px)。iris_d435i：hfov=1.5rad、848×480
/// → 2m 高横向半宽 ≈1.05m、4m 高 ≈2.11m（2026-09-12 覆盖计算的依据）。
inline double nadir_across_half_width(
  double alt_m, double hfov_rad, double img_w_px, double img_h_px)
{
  return std::fabs(alt_m) * std::tan(hfov_rad / 2.0) * (img_h_px / img_w_px);
}

/// 均匀布带下区内任一点到最近带的最大横向距离 = 带间距/2 = 跨度/(2(lanes-1))。
/// 覆盖判据：该值 ≤ nadir_across_half_width（带间无盲区），lanes==1 取半跨。
inline double max_cross_track_distance(double span, int lanes)
{
  const double s = std::fabs(span);
  if (lanes <= 1) {
    return s / 2.0;
  }
  return s / (2.0 * static_cast<double>(lanes - 1));
}

}  // namespace cuadc_route

#endif  // CUADC_MISSION__ROUTE_LOGIC_HPP_
