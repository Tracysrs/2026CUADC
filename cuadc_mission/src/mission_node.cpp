// =============================================================================
// CUADC 2026 多旋翼侦察与救援 · 任务状态机节点（M1 骨架）
//
// 设计依据（唯一权威方案）：01_设计/总体方案与执行计划.md（SSOT）
//   - §5.1 全生命周期状态（本文件状态枚举与其逐一对应，含 DONE 终态与
//     PILOT_OVERRIDE / ABORT 两条旁路）
//   - §5.4 参数基线（config/mission_params.yaml 默认值的出处）
//   - §10.2 坑清单（任务原点 / ENU 航向转换 / 锁航向 / odom 断流 ...）
// 参考实现：08_参考/code/2026code/状态机/cuadc_full_mission_node_3_v2_public.cpp
//   （按其 README 要求"先理解逻辑、再按 ROS 2 重新实现"，本文件为重写版骨架）
//
// M1 范围（能飞）：
//   [已实现] 准备五连 + LOCK_FRAME 独立态 + CommandTOL 起飞 + 预设航线
//            （时间插值平滑）+ 侦察段（拍照请求/确认握手）+ 返航
//            + 降落确认（≤0.30m + 速度门限 + 稳定 1.5s）+ 独立 DISARM 自动上锁
//   [fail-closed] SEARCH 正式模式 / ALIGN / RELEASE 属 M2/M3 范围：
//            骨架版触发即 fail_and_return，绝不带着未实现的对准/投放逻辑上天
//   [未开始] safety_monitor 独立进程、丢目标降级策略（M4）
//
// 安全铁律（SSOT §7）：
//   1. 每一版必须先过 SITL 再上真机；真机首测拆桨；
//   2. 飞手切走 GUIDED = 人工接管，本节点立即停发目标点并退让（绝不抢权）；
//   3. m1_no_vision_mode=true 仅为 M1 无视觉演练模式，M2 接入感知后必须关闭。
// =============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>

using namespace std::chrono_literals;

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr const char * kMissionVersion = "cuadc-m1-2026-09";

/// 三维点（本地 ENU 坐标系，米；z 向上，相对上电原点）
struct Point3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

/// 一段直线航线：按"距离/速度=时长"做时间插值，飞机追插值点而非直冲终点
struct Segment
{
  Point3 start;
  Point3 end;
  rclcpp::Time start_time;
  double duration_s = 1.0;
};

/// odom 历史样本：把视觉帧（带取帧时刻）夹逼/插值到取帧瞬间的位姿（P0.4，
/// 见 10_机载代码/时间同步设计.md；M2 的视觉-世界换算必须经过它）
struct OdomSample
{
  rclcpp::Time stamp;
  Point3 position;
  double yaw = 0.0;
};

/// 角度归一化到 (-pi, pi]
double normalize_angle(double x)
{
  return std::atan2(std::sin(x), std::cos(x));
}

/// 角度归一化到 [0, 360)
double normalize_degrees(double x)
{
  x = std::fmod(x, 360.0);
  return x < 0.0 ? x + 360.0 : x;
}

/// 罗盘角差 a-b，归一化到 [-180, 180]
double diff_degrees(double a, double b)
{
  double d = std::fmod(a - b, 360.0);
  if (d > 180.0) {
    d -= 360.0;
  }
  if (d < -180.0) {
    d += 360.0;
  }
  return d;
}

double distance_xy(const Point3 & a, const Point3 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double distance_xyz(const Point3 & a, const Point3 & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace

// =============================================================================
// 状态定义（SSOT §5.1 全生命周期）
// =============================================================================
enum class State
{
  // ---- 准备阶段（地面）----
  WAIT_FCU,         ///< 等飞控连接（MAVROS ↔ Pixhawk 握手）
  WAIT_NAV_STABLE,  ///< 等定位 + 罗盘可用
  LOCK_FRAME,       ///< 锁定任务坐标系：航向 2s 内变化 ≤2° 才放行（防罗盘漂移）
  PRESTREAM,        ///< 在起飞点预发 setpoint 1.5s（切 GUIDED 前必须已有目标流）
  WAIT_GUIDED,      ///< 等飞手切 GUIDED（= 飞手最终确认）
  WAIT_ARM,         ///< 等解锁（默认飞手手动；auto_arm_on_guided 才自动）
  // ---- 任务主线 ----
  TAKEOFF,          ///< CommandTOL 起飞，0.9×目标高后切 setpoint
  SEARCH,           ///< 2.0m 蛇形搜索；M1 演练=飞预设航线，正式=等感知锁定独立筒
  ALIGN,            ///< 粗对准（M3：目标锁定五步之后，fail-closed）
  RELEASE,          ///< 八门控投放 ×2（M3，fail-closed）
  // ---- 侦察段 ----
  RECON_CLIMB,      ///< 爬到侦察高度
  RECON_SURVEY,     ///< 蛇形 6 航点 + 每点拍照请求/确认（1s 超时兜底）
  // ---- 返航与收尾 ----
  RETURN_CLIMB,     ///< 先爬到返航高度，再横越场地
  RETURN_HOME,      ///< 返航到起飞点上方
  LAND,             ///< 降落交还飞控；落地判定 ≤0.30m + 速度门限 + 稳定 1.5s
  DISARM,           ///< 独立态：确认落地后自动上锁（规则 6.1.4 桨停转才有着陆分）
  DONE,             ///< 结束
  // ---- 旁路 ----
  PILOT_OVERRIDE,   ///< 飞手切走 GUIDED：程序立即退让并退出
  ABORT             ///< 致命故障：能降则降，不能则退出
};

std::string state_name(State s)
{
  switch (s) {
    case State::WAIT_FCU: return "WAIT_FCU";
    case State::WAIT_NAV_STABLE: return "WAIT_NAV_STABLE";
    case State::LOCK_FRAME: return "LOCK_FRAME";
    case State::PRESTREAM: return "PRESTREAM";
    case State::WAIT_GUIDED: return "WAIT_GUIDED";
    case State::WAIT_ARM: return "WAIT_ARM";
    case State::TAKEOFF: return "TAKEOFF";
    case State::SEARCH: return "SEARCH";
    case State::ALIGN: return "ALIGN";
    case State::RELEASE: return "RELEASE";
    case State::RECON_CLIMB: return "RECON_CLIMB";
    case State::RECON_SURVEY: return "RECON_SURVEY";
    case State::RETURN_CLIMB: return "RETURN_CLIMB";
    case State::RETURN_HOME: return "RETURN_HOME";
    case State::LAND: return "LAND";
    case State::DISARM: return "DISARM";
    case State::DONE: return "DONE";
    case State::PILOT_OVERRIDE: return "PILOT_OVERRIDE";
    case State::ABORT: return "ABORT";
  }
  return "UNKNOWN";
}

class CuadcMissionNode final : public rclcpp::Node
{
public:
  CuadcMissionNode()
  : Node("cuadc_mission")
  {
    declare_parameters();
    load_parameters();

    // ---- 订阅：飞控状态 / 定位 / 罗盘 / 侦察拍照确认 ----
    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", rclcpp::QoS(10).reliable(),
      std::bind(&CuadcMissionNode::state_callback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/mavros/local_position/odom", rclcpp::SensorDataQoS(),
      std::bind(&CuadcMissionNode::odom_callback, this, std::placeholders::_1));
    compass_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/mavros/global_position/compass_hdg", rclcpp::SensorDataQoS(),
      std::bind(&CuadcMissionNode::compass_callback, this, std::placeholders::_1));
    recon_ack_sub_ = create_subscription<std_msgs::msg::UInt32>(
      "/cuadc/recon/capture_done", 10,
      std::bind(&CuadcMissionNode::recon_ack_callback, this, std::placeholders::_1));

    // ---- 发布：目标点 / 状态广播 / 侦察拍照请求 ----
    setpoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mavros/setpoint_position/local", 10);
    mission_state_pub_ = create_publisher<std_msgs::msg::String>("/cuadc/mission_state", 10);
    recon_capture_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "/cuadc/recon/capture_request", 10);

    // ---- 服务客户端：全部异步发送 + 1s 节流，绝不阻塞状态机 ----
    takeoff_client_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
    land_client_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");
    arm_client_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");

    state_enter_time_ = now();
    mission_start_time_ = now();
    last_request_time_ = now() - rclcpp::Duration::from_seconds(2.0);
    last_odom_time_ = now();

    // 状态机心跳：50ms 一拍（20Hz）
    timer_ = create_wall_timer(50ms, std::bind(&CuadcMissionNode::tick, this));

    RCLCPP_INFO(get_logger(), "CUADC mission node ready [%s]", kMissionVersion);
    if (m1_no_vision_mode_) {
      RCLCPP_WARN(
        get_logger(),
        "M1 无视觉演练模式：SEARCH 飞完预设航线后跳过对准/投放，直接转侦察段。"
        "M2 接入感知后必须设 m1_no_vision_mode:=false");
    }
  }

private:
  // ===========================================================================
  // 参数：全部声明 + 下限保护（SSOT §10.2-6：启动即加载，YAML 与代码不脱节）
  // ===========================================================================
  void declare_parameters()
  {
    declare_parameter<bool>("m1_no_vision_mode", true);
    declare_parameter<bool>("auto_arm_on_guided", false);

    declare_parameter<double>("takeoff_alt_m", 4.0);
    declare_parameter<double>("search_alt_m", 2.0);
    declare_parameter<double>("recon_alt_m", 4.0);
    declare_parameter<double>("return_alt_m", 4.0);

    declare_parameter<double>("transit_speed_m_s", 4.0);
    declare_parameter<double>("search_speed_m_s", 2.0);
    declare_parameter<double>("recon_speed_m_s", 3.0);

    declare_parameter<double>("waypoint_accept_radius_m", 0.40);

    declare_parameter<double>("heading_lock_window_s", 2.0);
    declare_parameter<double>("heading_lock_tol_deg", 2.0);
    declare_parameter<double>("heading_lock_timeout_s", 30.0);

    declare_parameter<double>("prestream_hold_s", 1.5);
    declare_parameter<double>("takeoff_timeout_s", 60.0);
    declare_parameter<double>("mission_timeout_s", 240.0);
    declare_parameter<double>("odom_timeout_s", 1.0);
    declare_parameter<double>("recon_photo_timeout_s", 1.0);

    declare_parameter<double>("landing_confirm_s", 1.5);
    declare_parameter<double>("landing_alt_gate_m", 0.30);
    declare_parameter<double>("landing_hspeed_gate_m_s", 0.20);
    declare_parameter<double>("landing_vspeed_gate_m_s", 0.15);

    declare_parameter<double>("search_x_min_m", 6.0);
    declare_parameter<double>("search_x_max_m", 10.0);
    declare_parameter<double>("search_half_width_m", 2.0);
    declare_parameter<int>("search_lanes", 4);

    declare_parameter<double>("recon_x_m", 14.0);
    declare_parameter<double>("recon_lane_len_m", 2.0);
    declare_parameter<double>("recon_half_width_m", 2.0);
    declare_parameter<int>("recon_lanes", 3);

    // ---- 时间同步（P0.4，SSOT §4.3 容差照抄）----
    declare_parameter<double>("perception_max_delay_s", 1.5);   // 感知管线延迟上界
    declare_parameter<double>("odom_future_tol_s", 0.05);       // 未来戳容差
    declare_parameter<double>("odom_interp_max_gap_s", 0.2);    // 插值最大帧间隔
    declare_parameter<double>("odom_history_span_s", 3.0);      // odom 历史时间窗
  }

  void load_parameters()
  {
    m1_no_vision_mode_ = get_parameter("m1_no_vision_mode").as_bool();
    auto_arm_on_guided_ = get_parameter("auto_arm_on_guided").as_bool();

    takeoff_alt_m_ = std::max(1.0, get_parameter("takeoff_alt_m").as_double());
    search_alt_m_ = std::max(1.0, get_parameter("search_alt_m").as_double());
    recon_alt_m_ = std::max(2.0, get_parameter("recon_alt_m").as_double());
    return_alt_m_ = std::max(1.0, get_parameter("return_alt_m").as_double());

    transit_speed_m_s_ = std::max(0.2, get_parameter("transit_speed_m_s").as_double());
    search_speed_m_s_ = std::max(0.2, get_parameter("search_speed_m_s").as_double());
    recon_speed_m_s_ = std::max(0.2, get_parameter("recon_speed_m_s").as_double());
    waypoint_accept_radius_m_ = std::max(
      0.10, get_parameter("waypoint_accept_radius_m").as_double());

    heading_lock_window_s_ = std::max(0.5, get_parameter("heading_lock_window_s").as_double());
    heading_lock_tol_deg_ = std::max(0.5, get_parameter("heading_lock_tol_deg").as_double());
    heading_lock_timeout_s_ = std::max(5.0, get_parameter("heading_lock_timeout_s").as_double());

    prestream_hold_s_ = std::max(0.5, get_parameter("prestream_hold_s").as_double());
    takeoff_timeout_s_ = std::max(10.0, get_parameter("takeoff_timeout_s").as_double());
    mission_timeout_s_ = std::max(30.0, get_parameter("mission_timeout_s").as_double());
    odom_timeout_s_ = std::max(0.2, get_parameter("odom_timeout_s").as_double());
    recon_photo_timeout_s_ = std::max(0.2, get_parameter("recon_photo_timeout_s").as_double());

    landing_confirm_s_ = std::max(0.5, get_parameter("landing_confirm_s").as_double());
    landing_alt_gate_m_ = std::max(0.05, get_parameter("landing_alt_gate_m").as_double());
    landing_hspeed_gate_m_s_ = std::max(
      0.05, get_parameter("landing_hspeed_gate_m_s").as_double());
    landing_vspeed_gate_m_s_ = std::max(
      0.05, get_parameter("landing_vspeed_gate_m_s").as_double());

    search_x_min_m_ = get_parameter("search_x_min_m").as_double();
    search_x_max_m_ = get_parameter("search_x_max_m").as_double();
    search_half_width_m_ = std::max(0.5, get_parameter("search_half_width_m").as_double());
    search_lanes_ = std::max(1, static_cast<int>(get_parameter("search_lanes").as_int()));

    recon_x_m_ = get_parameter("recon_x_m").as_double();
    recon_lane_len_m_ = std::max(0.5, get_parameter("recon_lane_len_m").as_double());
    recon_half_width_m_ = std::max(0.5, get_parameter("recon_half_width_m").as_double());
    recon_lanes_ = std::max(1, static_cast<int>(get_parameter("recon_lanes").as_int()));

    perception_max_delay_s_ = std::max(
      0.2, get_parameter("perception_max_delay_s").as_double());
    odom_future_tol_s_ = std::max(0.01, get_parameter("odom_future_tol_s").as_double());
    odom_interp_max_gap_s_ = std::max(
      0.05, get_parameter("odom_interp_max_gap_s").as_double());
    odom_history_span_s_ = std::max(1.0, get_parameter("odom_history_span_s").as_double());
  }

  // ===========================================================================
  // 数据回调：数据一到就更新成员变量，供 tick() 随时查询
  // ===========================================================================
  void state_callback(const mavros_msgs::msg::State::SharedPtr msg)
  {
    const bool was_guided = guided_active_;
    fcu_state_ = *msg;
    guided_active_ = fcu_state_.connected && fcu_state_.mode == "GUIDED";
    // 飞手切走 GUIDED = 人工接管（SSOT §5.1 旁路）：立即停发目标点并退让
    if (was_guided && !guided_active_ && autonomous_state(state_)) {
      publish_setpoint_enabled_ = false;
      enter(State::PILOT_OVERRIDE);
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    position_ = Point3{
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z};
    horizontal_speed_m_s_ = std::hypot(
      msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    vertical_speed_m_s_ = msg->twist.twist.linear.z;
    have_odom_ = true;
    last_odom_time_ = now();

    // P0.4：维护 odom 历史供视觉帧时间插值（M2 消费，见 时间同步设计.md）
    OdomSample sample;
    // 统一换成节点时钟类型再比较/相减，避免 rclcpp::Time 因时钟类型不同抛异常
    sample.stamp = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());
    sample.position = position_;
    const auto & q = msg->pose.pose.orientation;
    sample.yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    odom_history_.push_back(sample);
    // 双重裁剪：按时间窗裁 + 硬上限兜底（防时间戳异常时不收敛）
    const rclcpp::Time cutoff =
      sample.stamp - rclcpp::Duration::from_seconds(odom_history_span_s_);
    while (!odom_history_.empty() && odom_history_.front().stamp < cutoff) {
      odom_history_.pop_front();
    }
    constexpr std::size_t kMaxOdomHistory = 600U;
    while (odom_history_.size() > kMaxOdomHistory) {
      odom_history_.pop_front();
    }
  }

  void compass_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (!std::isfinite(msg->data)) {
      return;
    }
    current_compass_deg_ = normalize_degrees(msg->data);
    // ENU 航向 = 90° − 罗盘航向（SSOT §10.2-1，坐标系列坑之首，勿改）
    current_heading_enu_ = normalize_angle((90.0 - current_compass_deg_) * kPi / 180.0);
    have_compass_ = true;
  }

  void recon_ack_callback(const std_msgs::msg::UInt32::SharedPtr msg)
  {
    if (std::find(recon_acks_.begin(), recon_acks_.end(), msg->data) == recon_acks_.end()) {
      recon_acks_.push_back(msg->data);
    }
  }

  // ===========================================================================
  // 坐标系与航线（SSOT §10.2-1：起飞前锁死任务原点与航向，全程不转机头）
  // ===========================================================================

  /// 场地坐标（原点=起飞点，x 朝锁定的机头方向，y 向左）→ 本地 ENU
  Point3 field_to_local(double x, double y, double rel_z) const
  {
    const Point3 origin = home_.value_or(Point3{});
    const double c = std::cos(mission_yaw_);
    const double s = std::sin(mission_yaw_);
    return Point3{origin.x + c * x - s * y, origin.y + s * x + c * y, origin.z + rel_z};
  }

  /// P0.4：把时间戳 t 夹逼/插值到 odom 历史上的位姿（SSOT §4.3 容差参数照抄）。
  /// 返回 nullopt 的三种情况：帧太旧（超感知管线延迟上界）/ 未来戳超容差 /
  /// 包围两帧间隔过大（odom 断流）。拒绝即丢弃该帧，绝不外推。
  std::optional<OdomSample> interpolate_odom(const rclcpp::Time & t) const
  {
    if (odom_history_.empty()) {
      return std::nullopt;
    }
    const OdomSample & newest = odom_history_.back();
    const OdomSample & oldest = odom_history_.front();
    if ((t - newest.stamp).seconds() > odom_future_tol_s_) {
      return std::nullopt;  // 未来戳超容差
    }
    if ((newest.stamp - t).seconds() > perception_max_delay_s_) {
      return std::nullopt;  // 帧太旧
    }
    if (t >= newest.stamp) {
      return newest;  // 夹逼上界（延迟小于一拍）
    }
    if (t <= oldest.stamp) {
      return oldest;  // 夹逼下界
    }
    for (auto it = odom_history_.begin(); it + 1 != odom_history_.end(); ++it) {
      const OdomSample & a = *it;
      const OdomSample & b = *(it + 1);
      if (t >= a.stamp && t <= b.stamp) {
        const double gap = (b.stamp - a.stamp).seconds();
        if (gap > odom_interp_max_gap_s_) {
          return std::nullopt;  // 断流段内不插值
        }
        const double r = (t - a.stamp).seconds() / std::max(1e-6, gap);
        const double dyaw = normalize_angle(b.yaw - a.yaw);
        return OdomSample{
          t,
          Point3{
            a.position.x + r * (b.position.x - a.position.x),
            a.position.y + r * (b.position.y - a.position.y),
            a.position.z + r * (b.position.z - a.position.z)},
          normalize_angle(a.yaw + r * dyaw)};
      }
    }
    return std::nullopt;
  }

  /// 把机体系目标（x前 y左 z上）按"取帧瞬间"的位姿换算到本地 ENU（M2 消费）。
  /// 相机-机体外参与投放口偏置在 M3 标定后叠加，本函数只做插值位姿的旋转平移。
  Point3 body_to_local_at(const OdomSample & ref, const Point3 & body) const
  {
    const double c = std::cos(ref.yaw);
    const double s = std::sin(ref.yaw);
    return Point3{
      ref.position.x + c * body.x - s * body.y,
      ref.position.y + s * body.x + c * body.y,
      ref.position.z + body.z};
  }

  /// 锁定任务坐标系。注意 EKF origin ≠ 起飞点，任务原点必须取起飞瞬间 odom
  void lock_frame()
  {
    home_ = position_;
    mission_yaw_ = current_heading_enu_;
    locked_compass_deg_ = current_compass_deg_;
    yaw_qz_ = std::sin(mission_yaw_ * 0.5);
    yaw_qw_ = std::cos(mission_yaw_ * 0.5);
    target_ = *home_;
    frame_locked_ = true;
    build_search_route();
    build_recon_route();
    RCLCPP_INFO(
      get_logger(), "任务坐标系已锁定: home=(%.2f, %.2f, %.2f) 航向=%.1f°",
      home_->x, home_->y, home_->z, locked_compass_deg_);
  }

  /// 弓字形搜索航线：search_lanes 条平行带，奇偶带交替方向
  void build_search_route()
  {
    search_route_.clear();
    const double x0 = std::min(search_x_min_m_, search_x_max_m_);
    const double x1 = std::max(search_x_min_m_, search_x_max_m_);
    for (int lane = 0; lane < search_lanes_; ++lane) {
      const double t = search_lanes_ == 1 ?
        0.5 : static_cast<double>(lane) / static_cast<double>(search_lanes_ - 1);
      const double y = -search_half_width_m_ + 2.0 * search_half_width_m_ * t;
      const bool forward = lane % 2 == 0;
      search_route_.push_back(field_to_local(forward ? x0 : x1, y, search_alt_m_));
      search_route_.push_back(field_to_local(forward ? x1 : x0, y, search_alt_m_));
    }
  }

  /// 侦察航线：recon_lanes 条带 × 两端点 = 2×recon_lanes 个航点（默认 6，SSOT §5.1）
  void build_recon_route()
  {
    recon_route_.clear();
    const double x0 = recon_x_m_;
    const double x1 = recon_x_m_ + recon_lane_len_m_;
    for (int lane = 0; lane < recon_lanes_; ++lane) {
      const double t = recon_lanes_ == 1 ?
        0.5 : static_cast<double>(lane) / static_cast<double>(recon_lanes_ - 1);
      const double y = -recon_half_width_m_ + 2.0 * recon_half_width_m_ * t;
      const bool forward = lane % 2 == 0;
      recon_route_.push_back(field_to_local(forward ? x0 : x1, y, recon_alt_m_));
      recon_route_.push_back(field_to_local(forward ? x1 : x0, y, recon_alt_m_));
    }
  }

  // ===========================================================================
  // 主状态机：50ms 一拍（20Hz）。每拍只回答三个问题：
  //   现在在哪个状态？该干什么？干完了吗（干完就 enter 下一态）？
  // ===========================================================================
  void tick()
  {
    check_service_results();
    if (publish_setpoint_enabled_ && frame_locked_) {
      publish_setpoint();
    }

    // 任务总超时看门狗（SSOT §5.4：240s，比赛 5 分钟）；
    // 一旦进入返航/降落段不再受它约束，让飞机安全回家
    if (mission_started_ && !returning_or_landed() &&
      (now() - mission_start_time_).seconds() > mission_timeout_s_)
    {
      fail_and_return("Mission timeout");
      return;
    }

    switch (state_) {
      case State::WAIT_FCU:
        publish_setpoint_enabled_ = false;
        if (fcu_state_.connected) {
          enter(State::WAIT_NAV_STABLE);
        }
        break;

      case State::WAIT_NAV_STABLE:
        publish_setpoint_enabled_ = false;
        if (!fcu_state_.connected) {
          enter(State::WAIT_FCU);
        } else if (fcu_state_.armed) {
          fail("Aircraft armed before mission frame lock");
        } else if (navigation_ready()) {
          enter(State::LOCK_FRAME);
        }
        break;

      case State::LOCK_FRAME:
        publish_setpoint_enabled_ = false;
        if (!fcu_state_.connected) {
          enter(State::WAIT_FCU);
        } else if (fcu_state_.armed) {
          fail("Aircraft armed before mission frame lock");
        } else if (!navigation_ready()) {
          heading_lock_since_.reset();  // 定位断了重新计时
        } else if (heading_locked()) {
          lock_frame();
          publish_setpoint_enabled_ = true;
          enter(State::PRESTREAM);
        } else if ((now() - state_enter_time_).seconds() > heading_lock_timeout_s_) {
          fail("Heading drift beyond lock tolerance (罗盘漂移？检查校准与干扰)");
        }
        break;

      case State::PRESTREAM:
        target_ = home_.value_or(position_);
        if (!navigation_ready()) {
          fail("Navigation lost during prestream");
        } else if ((now() - state_enter_time_).seconds() >= prestream_hold_s_) {
          enter(State::WAIT_GUIDED);
        }
        break;

      case State::WAIT_GUIDED:
        target_ = home_.value_or(position_);
        if (guided_active_) {
          enter(State::WAIT_ARM);
        }
        break;

      case State::WAIT_ARM:
        target_ = home_.value_or(position_);
        if (!guided_active_) {
          enter(State::WAIT_GUIDED);
        } else if (fcu_state_.armed) {
          mission_started_ = true;
          mission_start_time_ = now();
          publish_setpoint_enabled_ = false;  // 起飞交 CommandTOL，不与之抢权
          enter(State::TAKEOFF);
        } else if (auto_arm_on_guided_) {
          request_arm(true);
        }
        break;

      case State::TAKEOFF:
        publish_setpoint_enabled_ = false;
        if (!flight_gate_ok()) {
          break;
        }
        if (!takeoff_sent_) {
          request_takeoff();
        }
        if (relative_altitude() >= takeoff_alt_m_ * 0.90) {
          publish_setpoint_enabled_ = true;
          start_search();
        } else if ((now() - state_enter_time_).seconds() > takeoff_timeout_s_) {
          fail_and_return("Takeoff timeout");
        }
        break;

      case State::SEARCH:
        if (flight_gate_ok()) {
          update_search();
        }
        break;

      case State::ALIGN:
      case State::RELEASE:
        // TODO(M3)：目标锁定五步（SSOT §5.2）+ 八门控投放 + 标定偏置注入。
        // 骨架版 fail-closed：正常流转不会进入这两个状态。
        fail_and_return("ALIGN/RELEASE not implemented until M3");
        break;

      case State::RECON_CLIMB:
        if (flight_gate_ok()) {
          target_ = sample_segment();
          if (segment_complete(waypoint_accept_radius_m_)) {
            recon_index_ = 0U;
            photo_requested_ = false;
            if (recon_route_.empty()) {
              start_return();
            } else {
              start_segment(position_, recon_route_.front(), recon_speed_m_s_);
              enter(State::RECON_SURVEY);
            }
          }
        }
        break;

      case State::RECON_SURVEY:
        if (flight_gate_ok()) {
          update_recon_survey();
        }
        break;

      case State::RETURN_CLIMB:
        if (flight_gate_ok()) {
          target_ = sample_segment();
          if (segment_complete(waypoint_accept_radius_m_)) {
            start_segment(
              position_, Point3{home_->x, home_->y, home_->z + return_alt_m_},
              transit_speed_m_s_);
            enter(State::RETURN_HOME);
          }
        }
        break;

      case State::RETURN_HOME:
        if (flight_gate_ok()) {
          target_ = sample_segment();
          if (segment_complete(std::max(waypoint_accept_radius_m_, 0.5))) {
            publish_setpoint_enabled_ = false;  // 降落交还飞控
            enter(State::LAND);
          }
        }
        break;

      case State::LAND:
        publish_setpoint_enabled_ = false;
        if (landing_candidate()) {
          if (!landing_stable_since_.has_value()) {
            landing_stable_since_ = now();
          } else if ((now() - *landing_stable_since_).seconds() >= landing_confirm_s_) {
            enter(fcu_state_.armed ? State::DISARM : State::DONE);
          }
        } else {
          landing_stable_since_.reset();
          if (fcu_state_.armed) {
            request_land();
          }
        }
        break;

      case State::DISARM:
        publish_setpoint_enabled_ = false;
        if (!fcu_state_.armed) {
          enter(State::DONE);
        } else if (landing_candidate()) {
          request_arm(false);
        } else {
          enter(State::LAND);
        }
        break;

      case State::DONE:
        publish_setpoint_enabled_ = false;
        if (!done_logged_) {
          done_logged_ = true;
          std::string summary = "任务结束: 拍照确认数=" + std::to_string(recon_acks_.size());
          if (mission_failed_) {
            summary += " | 失败原因: " + terminal_reason_;
          }
          RCLCPP_INFO(get_logger(), "%s", summary.c_str());
        }
        if ((now() - state_enter_time_).seconds() > 1.0) {
          rclcpp::shutdown();
        }
        break;

      case State::PILOT_OVERRIDE:
        publish_setpoint_enabled_ = false;
        // 控制权已在飞手手里，程序体面退场
        if ((now() - state_enter_time_).seconds() > 1.0) {
          rclcpp::shutdown();
        }
        break;

      case State::ABORT:
        publish_setpoint_enabled_ = false;
        if (fcu_state_.armed) {
          enter(State::LAND);
        } else {
          rclcpp::shutdown();
        }
        break;
    }
  }

  // ===========================================================================
  // 各阶段更新函数
  // ===========================================================================
  void start_search()
  {
    if (search_route_.empty()) {
      fail_and_return("Search route is empty");
      return;
    }
    search_index_ = 0U;
    start_segment(position_, search_route_.front(), search_speed_m_s_);
    enter(State::SEARCH);
  }

  void update_search()
  {
    // TODO(M2)：感知接入后在这里做"目标锁定五步"（SSOT §5.2）：
    //   ≥3 独立筒确认（≥5 帧 + 位置 EMA + 独立性门控 + 直径先验 + 排名稳定 0.8s）
    //   → 冻结目标集 → 取当前载荷对应目标进 ALIGN。
    //   视觉-世界换算必须走 interpolate_odom() + body_to_local_at()（P0.4 已就绪，
    //   见 时间同步设计.md），禁止直接用当前 position_ 换算。
    // 正式模式感知未接入 = fail-closed：
    if (!m1_no_vision_mode_) {
      fail_and_return("SEARCH requires perception (TODO M2)");
      return;
    }
    // M1 演练模式：沿预设航线飞行，走完直接转侦察段
    target_ = sample_segment();
    if (!segment_complete(waypoint_accept_radius_m_)) {
      return;
    }
    ++search_index_;
    if (search_index_ < search_route_.size()) {
      start_segment(position_, search_route_[search_index_], search_speed_m_s_);
      return;
    }
    RCLCPP_WARN(
      get_logger(), "M1 演练：搜索航线完成，跳过对准/投放（M3 范围），转侦察段");
    start_recon_climb();
  }

  void start_recon_climb()
  {
    start_segment(
      position_, Point3{position_.x, position_.y, home_->z + recon_alt_m_},
      transit_speed_m_s_);
    enter(State::RECON_CLIMB);
  }

  void update_recon_survey()
  {
    target_ = sample_segment();
    if (!segment_complete(waypoint_accept_radius_m_)) {
      return;
    }
    // 到达视点：请求拍照（只发一次，防重复）
    if (!photo_requested_) {
      request_recon_photo(recon_index_);
      photo_requested_ = true;
      photo_request_time_ = now();
      return;
    }
    // 等拍照确认；侦察节点未在线时按 recon_photo_timeout_s 兜底放行
    if (!recon_ack_received(recon_index_) &&
      (now() - photo_request_time_).seconds() < recon_photo_timeout_s_)
    {
      return;
    }
    photo_requested_ = false;
    ++recon_index_;
    if (recon_index_ >= recon_route_.size()) {
      start_return();
      return;
    }
    start_segment(position_, recon_route_[recon_index_], recon_speed_m_s_);
  }

  void start_return()
  {
    if (!home_.has_value()) {
      fail("Return requested without home");
      return;
    }
    // 先爬到返航高度再横越场地（SSOT §5.1 RETURN_CLIMB）
    start_segment(
      position_, Point3{position_.x, position_.y, home_->z + return_alt_m_},
      transit_speed_m_s_);
    enter(State::RETURN_CLIMB);
  }

  // ===========================================================================
  // 航段与航线平滑：时间插值 + smoothstep 缓入缓出
  // ===========================================================================
  void start_segment(const Point3 & start, const Point3 & end, double speed)
  {
    const double duration = std::max(0.5, distance_xyz(start, end) / std::max(0.1, speed));
    segment_ = Segment{start, end, now(), duration};
  }

  /// 当前段的插值点：进度 t 用 smoothstep = t²(3−2t)，段首加速段尾减速
  Point3 sample_segment() const
  {
    const double t = std::clamp(
      (now() - segment_.start_time).seconds() / segment_.duration_s, 0.0, 1.0);
    const double smooth = t * t * (3.0 - 2.0 * t);
    return Point3{
      segment_.start.x + smooth * (segment_.end.x - segment_.start.x),
      segment_.start.y + smooth * (segment_.end.y - segment_.start.y),
      segment_.start.z + smooth * (segment_.end.z - segment_.start.z)};
  }

  /// 段完成 = 时间到 且 实际位置进入接受半径（双条件，防吹偏）
  bool segment_complete(double radius) const
  {
    return (now() - segment_.start_time).seconds() >= segment_.duration_s &&
      distance_xyz(position_, segment_.end) <= radius;
  }

  // ===========================================================================
  // 安全门禁（TODO(M4)：safety_monitor 独立进程做第二道防线）
  // ===========================================================================
  bool navigation_ready() const
  {
    return fcu_state_.connected && have_odom_ && have_compass_ && odom_fresh();
  }

  bool odom_fresh() const
  {
    return have_odom_ && (now() - last_odom_time_).seconds() <= odom_timeout_s_;
  }

  /// 每个飞行态更新前的安全门禁：掉连接/掉解锁 → 中止；GUIDED 丢 → 接管；
  /// 定位过期 → 有序返航
  bool flight_gate_ok()
  {
    if (!fcu_state_.connected || !fcu_state_.armed) {
      fail("FCU disconnected or disarmed during mission");
      return false;
    }
    if (!guided_active_) {
      enter(State::PILOT_OVERRIDE);
      return false;
    }
    if (!odom_fresh()) {
      fail_and_return("Odometry stale during mission");
      return false;
    }
    return true;
  }

  double relative_altitude() const
  {
    return home_.has_value() ? position_.z - home_->z : 0.0;
  }

  /// 自主飞行态清单：接管保护只在这些状态触发
  bool autonomous_state(State s) const
  {
    return s == State::WAIT_ARM || s == State::TAKEOFF || s == State::SEARCH ||
      s == State::ALIGN || s == State::RELEASE || s == State::RECON_CLIMB ||
      s == State::RECON_SURVEY || s == State::RETURN_CLIMB || s == State::RETURN_HOME;
  }

  bool returning_or_landed() const
  {
    return state_ == State::RETURN_CLIMB || state_ == State::RETURN_HOME ||
      state_ == State::LAND || state_ == State::DISARM || state_ == State::DONE ||
      state_ == State::PILOT_OVERRIDE || state_ == State::ABORT;
  }

  /// 落地判定（SSOT §5.4：≤0.30m + 速度门限；配合 LAND 态稳定 1.5s 确认）
  bool landing_candidate() const
  {
    return odom_fresh() && std::abs(relative_altitude()) <= landing_alt_gate_m_ &&
      horizontal_speed_m_s_ <= landing_hspeed_gate_m_s_ &&
      std::abs(vertical_speed_m_s_) <= landing_vspeed_gate_m_s_;
  }

  /// LOCK_FRAME 判定：航向在窗口期内漂移 ≤tol 才算锁定，超限则重新计时
  bool heading_locked()
  {
    const double drift = std::fabs(diff_degrees(current_compass_deg_, heading_lock_ref_deg_));
    if (drift > heading_lock_tol_deg_) {
      heading_lock_ref_deg_ = current_compass_deg_;
      heading_lock_since_ = now();
      return false;
    }
    if (!heading_lock_since_.has_value()) {
      heading_lock_ref_deg_ = current_compass_deg_;
      heading_lock_since_ = now();
      return false;
    }
    return (now() - *heading_lock_since_).seconds() >= heading_lock_window_s_;
  }

  // ===========================================================================
  // 目标点发布与飞控指令（全部异步 + 1s 节流，绝不阻塞状态机）
  // ===========================================================================
  void publish_setpoint()
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.pose.position.x = target_.x;
    msg.pose.position.y = target_.y;
    msg.pose.position.z = target_.z;
    msg.pose.orientation.z = yaw_qz_;  // 纯偏航四元数：全程锁头不转机头
    msg.pose.orientation.w = yaw_qw_;
    setpoint_pub_->publish(msg);
  }

  bool request_allowed() const
  {
    return (now() - last_request_time_).seconds() >= 1.0;
  }

  void request_takeoff()
  {
    if (!request_allowed() || takeoff_future_.valid() || !takeoff_client_->service_is_ready()) {
      return;
    }
    auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    req->altitude = static_cast<float>(takeoff_alt_m_);
    req->yaw = static_cast<float>(locked_compass_deg_);
    last_request_time_ = now();
    takeoff_future_ = takeoff_client_->async_send_request(req).future.share();
    takeoff_sent_ = true;
  }

  void request_land()
  {
    if (!request_allowed() || land_future_.valid() || !land_client_->service_is_ready()) {
      return;
    }
    auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    req->yaw = static_cast<float>(locked_compass_deg_);
    last_request_time_ = now();
    land_future_ = land_client_->async_send_request(req).future.share();
  }

  void request_arm(bool arm)
  {
    if (!request_allowed() || arm_future_.valid() || !arm_client_->service_is_ready()) {
      return;
    }
    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = arm;
    last_request_time_ = now();
    arm_future_ = arm_client_->async_send_request(req).future.share();
  }

  /// 非阻塞回收服务结果（wait_for(0s) 只看一眼绝不等待）；起飞被拒则下拍重试
  void check_service_results()
  {
    if (takeoff_future_.valid() && takeoff_future_.wait_for(0s) == std::future_status::ready) {
      const auto res = takeoff_future_.get();
      if (!res->success) {
        takeoff_sent_ = false;
      }
      takeoff_future_ = {};
    }
    if (land_future_.valid() && land_future_.wait_for(0s) == std::future_status::ready) {
      (void)land_future_.get();
      land_future_ = {};
    }
    if (arm_future_.valid() && arm_future_.wait_for(0s) == std::future_status::ready) {
      (void)arm_future_.get();
      arm_future_ = {};
    }
  }

  void request_recon_photo(std::size_t index)
  {
    geometry_msgs::msg::PointStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.point.x = static_cast<double>(index);
    if (index < recon_route_.size()) {
      msg.point.y = recon_route_[index].x;  // 视点坐标随请求附带，侦察节点取证用
      msg.point.z = recon_route_[index].y;
    }
    recon_capture_pub_->publish(msg);
  }

  bool recon_ack_received(std::size_t index) const
  {
    return std::find(
      recon_acks_.begin(), recon_acks_.end(),
      static_cast<std::uint32_t>(index)) != recon_acks_.end();
  }

  // ===========================================================================
  // 失败处理与状态切换
  // ===========================================================================

  /// 硬失败：立即降落（掉连接/掉解锁等没有返航余量的情况）
  void fail(const std::string & reason)
  {
    mission_failed_ = true;
    terminal_reason_ = reason;
    RCLCPP_ERROR(get_logger(), "MISSION_ABORT: %s", reason.c_str());
    enter(fcu_state_.armed ? State::LAND : State::ABORT);
  }

  /// 软失败：先有序返航再降落（能回家绝不原地掉）
  void fail_and_return(const std::string & reason)
  {
    mission_failed_ = true;
    terminal_reason_ = reason;
    RCLCPP_ERROR(get_logger(), "MISSION_FAILURE: %s", reason.c_str());
    if (fcu_state_.armed && home_.has_value() && guided_active_ && odom_fresh()) {
      start_return();
    } else {
      enter(fcu_state_.armed ? State::LAND : State::ABORT);
    }
  }

  /// 状态切换唯一入口：记录进入时间 + 广播 + 日志（排障全靠这条轨迹）
  void enter(State next)
  {
    if (state_ == next) {
      return;
    }
    state_ = next;
    state_enter_time_ = now();
    std_msgs::msg::String msg;
    msg.data = state_name(state_);
    mission_state_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "STATE -> %s", msg.data.c_str());
  }

  // ===========================================================================
  // 成员变量
  // ===========================================================================

  // ---- 通信 ----
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr compass_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr recon_ack_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr recon_capture_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture takeoff_future_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture land_future_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture arm_future_;

  // ---- 状态机运行时 ----
  State state_ = State::WAIT_FCU;
  mavros_msgs::msg::State fcu_state_;
  Point3 position_;
  Point3 target_;
  Segment segment_;
  std::optional<Point3> home_;
  std::vector<Point3> search_route_;
  std::vector<Point3> recon_route_;
  std::vector<std::uint32_t> recon_acks_;
  std::deque<OdomSample> odom_history_;
  std::optional<rclcpp::Time> landing_stable_since_;
  std::optional<rclcpp::Time> heading_lock_since_;

  // ---- 布尔标志 ----
  bool guided_active_ = false;
  bool have_odom_ = false;
  bool have_compass_ = false;
  bool frame_locked_ = false;
  bool publish_setpoint_enabled_ = false;
  bool mission_started_ = false;
  bool mission_failed_ = false;
  bool done_logged_ = false;
  bool takeoff_sent_ = false;
  bool photo_requested_ = false;
  bool m1_no_vision_mode_ = true;
  bool auto_arm_on_guided_ = false;

  // ---- 计数器 ----
  std::size_t search_index_ = 0U;
  std::size_t recon_index_ = 0U;
  int search_lanes_ = 4;
  int recon_lanes_ = 3;

  // ---- 参数缓存（默认值 = SSOT §5.4 基线）----
  double takeoff_alt_m_ = 4.0;
  double search_alt_m_ = 2.0;
  double recon_alt_m_ = 4.0;
  double return_alt_m_ = 4.0;
  double transit_speed_m_s_ = 4.0;
  double search_speed_m_s_ = 2.0;
  double recon_speed_m_s_ = 3.0;
  double waypoint_accept_radius_m_ = 0.40;
  double heading_lock_window_s_ = 2.0;
  double heading_lock_tol_deg_ = 2.0;
  double heading_lock_timeout_s_ = 30.0;
  double prestream_hold_s_ = 1.5;
  double takeoff_timeout_s_ = 60.0;
  double mission_timeout_s_ = 240.0;
  double odom_timeout_s_ = 1.0;
  double recon_photo_timeout_s_ = 1.0;
  double landing_confirm_s_ = 1.5;
  double landing_alt_gate_m_ = 0.30;
  double landing_hspeed_gate_m_s_ = 0.20;
  double landing_vspeed_gate_m_s_ = 0.15;
  double search_x_min_m_ = 6.0;
  double search_x_max_m_ = 10.0;
  double search_half_width_m_ = 2.0;
  double recon_x_m_ = 14.0;
  double recon_lane_len_m_ = 2.0;
  double recon_half_width_m_ = 2.0;
  double perception_max_delay_s_ = 1.5;
  double odom_future_tol_s_ = 0.05;
  double odom_interp_max_gap_s_ = 0.2;
  double odom_history_span_s_ = 3.0;

  // ---- 坐标系缓存 ----
  double horizontal_speed_m_s_ = 0.0;
  double vertical_speed_m_s_ = 0.0;
  double current_compass_deg_ = 0.0;
  double current_heading_enu_ = 0.0;
  double mission_yaw_ = 0.0;
  double locked_compass_deg_ = 0.0;
  double heading_lock_ref_deg_ = 0.0;
  double yaw_qz_ = 0.0;
  double yaw_qw_ = 1.0;

  // ---- 时间戳 ----
  rclcpp::Time state_enter_time_;
  rclcpp::Time mission_start_time_;
  rclcpp::Time last_request_time_;
  rclcpp::Time last_odom_time_;
  rclcpp::Time photo_request_time_;

  std::string terminal_reason_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CuadcMissionNode>());
  rclcpp::shutdown();
  return 0;
}
