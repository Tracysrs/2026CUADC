#include <cmath>
#include <map>
#include <sstream>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

struct Bucket {
  double x;
  double y;
  double radius;
  int score;
};

// =============================================================================
// 虚拟投放判定节点（仿真判分，真机不跑）
//
// 判定 = 服务调用瞬间的 odom 水平位置到桶心的距离：
//   ≤ 桶半径 → A 区满分；≤ b_zone_radius → B 区 50 分；否则无效。
// 2026-09-12 修订：
//   1. 桶心/半径/分值改为 ROS 参数，默认 = generated_scene.yaml（seed 2027）
//      的真实摆放——旧版硬编码 (30,-1.2)/(30,0)/(30,1.2) 与 world 模型脱节；
//      换 seed 时把 generated_scene.yaml 的 drop_targets 抄进参数即可。
//   2. target_bucket_id 默认空 = 按 odom 最近桶判定（桶间距 ≥1.98m 远大于
//      投放精度 0.5m，最近桶即目标桶，mission 节点无需逐次设置目标）；
//      非空时保留旧语义（显式指定判分桶）。
// =============================================================================
class VirtualDropJudge : public rclcpp::Node {
public:
  VirtualDropJudge() : Node("virtual_drop_judge_node") {
    // 默认值 = 03_工具/仿真环境/cuadc_rescue_sim/config/generated_scene.yaml
    load_bucket("drop_1", 28.44, 0.02, 0.075, 500);
    load_bucket("drop_2", 30.05, 2.56, 0.10, 300);
    load_bucket("drop_3", 28.37, -1.96, 0.125, 100);

    target_bucket_id_ = declare_parameter<std::string>("target_bucket_id", "");
    b_zone_radius_ = declare_parameter<double>("b_zone_radius", 0.5);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/mavros/local_position/odom",
      rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        have_odom_ = true;
      });

    release_srv_ = create_service<std_srvs::srv::Trigger>(
      "/drop_controller/release",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        handle_release(response);
      });

    RCLCPP_INFO(get_logger(),
      "Virtual drop judge ready on /drop_controller/release, %zu buckets, "
      "target=%s(空=最近桶)",
      buckets_.size(), target_bucket_id_.c_str());
  }

private:
  void load_bucket(const std::string & id, double dx, double dy,
    double dr, int dscore)
  {
    Bucket b{};
    b.x = declare_parameter<double>(id + ".x", dx);
    b.y = declare_parameter<double>(id + ".y", dy);
    b.radius = declare_parameter<double>(id + ".radius", dr);
    b.score = declare_parameter<int>(id + ".score", dscore);
    buckets_[id] = b;
  }

  const Bucket * pick_target(std::string & picked_id) const {
    const std::string explicit_id = get_parameter("target_bucket_id").as_string();
    if (!explicit_id.empty()) {
      const auto it = buckets_.find(explicit_id);
      picked_id = explicit_id;
      return it != buckets_.end() ? &it->second : nullptr;
    }
    // 最近桶判定：投放时飞机悬停在目标桶正上方，桶间距远大于投放误差
    const Bucket * best = nullptr;
    double best_d2 = 1e18;
    for (const auto & [id, b] : buckets_) {
      const double d2 = (x_ - b.x) * (x_ - b.x) + (y_ - b.y) * (y_ - b.y);
      if (d2 < best_d2) {
        best_d2 = d2;
        best = &b;
        picked_id = id;
      }
    }
    return best;
  }

  void handle_release(const std_srvs::srv::Trigger::Response::SharedPtr &response) {
    release_count_++;

    if (!have_odom_) {
      response->success = false;
      response->message = "No odometry received yet";
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      return;
    }

    std::string picked_id;
    const Bucket * bucket = pick_target(picked_id);
    if (bucket == nullptr) {
      response->success = false;
      response->message = "Unknown target bucket: " + picked_id;
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      return;
    }

    const double dx = x_ - bucket->x;
    const double dy = y_ - bucket->y;
    const double error = std::hypot(dx, dy);

    std::string zone = "invalid";
    int score = 0;
    if (error <= bucket->radius) {
      zone = "A";
      score = bucket->score;
    } else if (error <= b_zone_radius_) {
      zone = "B";
      score = 50;
    }

    std::ostringstream msg;
    msg << "release=" << release_count_
        << " target=" << picked_id
        << " drone=(" << x_ << "," << y_ << ")"
        << " bucket=(" << bucket->x << "," << bucket->y << ")"
        << " error=" << error
        << " zone=" << zone
        << " score=" << score;

    response->success = score > 0;
    response->message = msg.str();
    total_score_ += score;
    RCLCPP_INFO(get_logger(), "%s (累计=%d)", response->message.c_str(), total_score_);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr release_srv_;

  std::map<std::string, Bucket> buckets_;
  std::string target_bucket_id_;
  double b_zone_radius_ = 0.5;
  double x_ = 0.0;
  double y_ = 0.0;
  bool have_odom_ = false;
  int release_count_ = 0;
  int total_score_ = 0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VirtualDropJudge>());
  rclcpp::shutdown();
  return 0;
}
