// =============================================================================
// odom_interp.hpp —— odom 历史夹逼/插值纯函数（无 ROS 依赖，可离线单测）
//
// P0.4 时间同步的判定核心（时间同步设计.md §3）：拒绝即丢弃，绝不外推。
// 2026-09-20 自 mission_node.cpp interpolate_odom() 抽出（行为逐分支保持，
// 含等号边界：端点恰等返回该端点、间隔严格大于才拒）。抽取动机：§15-A2
// 时间域回归测试（插值等号/大间隔/时间回跳）需要无 ROS 纯函数。
// Python 镜像 = tools/odom_interp.py；测试 = test/test_odom_interp.cpp 与
// tools/test_odom_interp.py 双端一一镜像。
// =============================================================================

#ifndef CUADC_MISSION__ODOM_INTERP_HPP_
#define CUADC_MISSION__ODOM_INTERP_HPP_

#include <cmath>
#include <deque>

namespace cuadc_odom {

struct Sample {
  double t = 0.0;    ///< 采样时刻（秒；单一时钟域内比较，跨域先换算）
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;
};

struct InterpParams {
  double future_tol_s = 0.05;   ///< 未来戳容差（SSOT §4.3）
  double max_delay_s = 1.5;     ///< 感知管线延迟上界
  double max_gap_s = 0.2;       ///< 插值最大帧间隔（断流段拒绝）
};

enum class InterpStatus { kOk, kEmpty, kFutureStamp, kTooOld, kGapBreak };

struct InterpResult {
  InterpStatus status = InterpStatus::kEmpty;
  Sample out;
};

inline double normalize_angle(double x)
{
  return std::atan2(std::sin(x), std::cos(x));
}

/// 把时刻 t 夹逼/插值到 odom 历史上。边界语义（test_odom_interp 锁死，勿改）：
///   空历史→kEmpty；t 比最新超 future_tol（严格>）→kFutureStamp；t 比最老
///   旧超 max_delay（严格>）→kTooOld；t≥最新→返回最新（夹逼上界，恰等含）；
///   t≤最老→返回最老（夹逼下界，恰等含——超老但在延迟窗内不丢弃）；
///   包围两帧 t≥a && t≤b（双闭）：间隔>max_gap（严格>，恰等放行）→kGapBreak；
///   否则线性插值 + yaw 最短角差插值后再归一化。
inline InterpResult interpolate(const std::deque<Sample> & hist, double t,
  const InterpParams & p)
{
  if (hist.empty()) {
    return {InterpStatus::kEmpty, {}};
  }
  const Sample & newest = hist.back();
  const Sample & oldest = hist.front();
  if (t - newest.t > p.future_tol_s) {
    return {InterpStatus::kFutureStamp, {}};   // 未来戳超容差
  }
  if (newest.t - t > p.max_delay_s) {
    return {InterpStatus::kTooOld, {}};        // 帧太旧
  }
  if (t >= newest.t) {
    return {InterpStatus::kOk, newest};        // 夹逼上界（延迟小于一拍）
  }
  if (t <= oldest.t) {
    return {InterpStatus::kOk, oldest};        // 夹逼下界
  }
  for (auto it = hist.begin(); it + 1 != hist.end(); ++it) {
    const Sample & a = *it;
    const Sample & b = *(it + 1);
    if (t >= a.t && t <= b.t) {
      const double gap = b.t - a.t;
      if (gap > p.max_gap_s) {
        return {InterpStatus::kGapBreak, {}};  // 断流段内不插值
      }
      const double r = (t - a.t) / std::max(1e-6, gap);
      const double dyaw = normalize_angle(b.yaw - a.yaw);
      return {InterpStatus::kOk,
        Sample{t,
          a.x + r * (b.x - a.x), a.y + r * (b.y - a.y), a.z + r * (b.z - a.z),
          normalize_angle(a.yaw + r * dyaw)}};
    }
  }
  return {InterpStatus::kGapBreak, {}};
}

}  // namespace cuadc_odom

#endif  // CUADC_MISSION__ODOM_INTERP_HPP_
