// =============================================================================
// drop_logic.hpp —— 投放决策核心（纯逻辑，header-only，仅依赖 std，无 ROS）
//
// 这是 cuadc_mission/tools/drop_logic.py 的 C++17 逐函数移植（该 Python 版是
// "已验证规范"：29 项单元测试 + 闭环蒙特卡洛全绿）。两边必须同步维护：
// 改任何阈值/判据，先改 Python 版并跑绿测试，再同步到这里。
//
// 链条（SSOT §5.2/§5.4/§6 + HIT 冠军版经验）：
//   SEARCH   BucketMap：世界系检测 → 关联/EMA/确认 → 独立性强制合并 → 直径对号
//            → 排名稳定 0.8s → 冻结目标集（冻结参考此后绝不改写）
//   ALIGN    TargetTracker：活动估计（新鲜度=反盲投资格）；粗 0.15 → 精 0.08m；
//            丢视觉只允许一次重捕获，再丢 = 弃桶（禁止按冻结坐标盲投）
//   RELEASE  冻结瞄准点（冻结时活动估计 + 标定投放口偏置 + 弹道前移）
//            → 八门控连续稳定 0.8s + 保持 1.5s（6s 超时弃桶）→ 单发舵机 0.7s
//
// 输入输出全部是世界系（本地 ENU，米）；视觉帧必须先经 mission_node 的
// interpolate_odom() + body_to_local_at()（P0.4 取帧时刻插值）再喂进来。
// =============================================================================

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

namespace cuadc_drop {

constexpr double kG = 9.81;

// ---------------------------------------------------------------------------
// SEARCH 段：桶航迹地图 + 锁定五步
// ---------------------------------------------------------------------------
struct LockParams {
  int min_confirm_frames = 5;           ///< 锁定五步①：≥5 帧确认
  double assoc_gate_m = 0.50;           ///< 检测-航迹关联门（< 筒间距）
  double ema_alpha = 0.25;              ///< 位置 EMA（SSOT §5.2）
  int diameter_window = 9;              ///< 直径中位窗
  double jitter_gate_m = 0.15;          ///< 确认时位置抖动门（世界系 std）
  double fresh_window_s = 1.0;          ///< SEARCH 段航迹过期时间
  double min_spacing_m = 0.20;          ///< 独立性门控：水平距
  double min_diameter_diff_m = 0.025;   ///< 独立性门控：直径差
  double diameter_min_m = 0.08;         ///< 物理先验（免费一致性校验）
  double diameter_max_m = 0.35;
  double nominal_diameters[3] = {0.15, 0.20, 0.25};
  double diameter_match_tol_m = 0.035;  ///< 对号容差
  double rank_stable_s = 0.8;           ///< 锁定五步⑤：排名稳定才锁
  int preferred_targets = 3;            ///< 正式模式需 3 独立分类筒
  int degraded_classified = 2;          ///< 降级模式（"2 筒 + 1 未知"）
  double blacklist_radius_m = 0.25;     ///< 已投筒拉黑半径
  double blacklist_window_s = 120.0;    ///< 拉黑时长
};

struct BucketTrack {
  int tid = 0;
  double x = 0.0;
  double y = 0.0;
  double diameter = 0.0;                            ///< 中位数
  std::deque<double> diam_samples;                  ///< 最近 N 次直径样本
  std::deque<std::pair<double, double>> positions;  ///< 最近 EMA 位置（算抖动）
  int confirms = 1;
  double first_seen = 0.0;
  double last_seen = 0.0;
};

struct FrozenTarget {
  int tid = -1;
  double frozen_x = 0.0;          ///< 锁死后绝不被改写
  double frozen_y = 0.0;
  double frozen_diameter = 0.0;
  int diameter_class = -1;        ///< 0/1/2 → 15/20/25cm；-1 = 未知
  double working_x = 0.0;         ///< 活动估计（重捕获只改这里）
  double working_y = 0.0;
  double last_vision_t = 0.0;
  int reacquires_used = 0;
  bool lost_since_valid = false;
  double lost_since = 0.0;
};

struct LockResult {
  bool ok = false;
  std::string reason;
  std::vector<FrozenTarget> targets;
};

enum class UpdateResult { kAssociated, kNewTrack, kRejectedDiameter };

inline double stddev_pop(const std::deque<double> & v)
{
  const std::size_t n = v.size();
  if (n < 2) {
    return 0.0;
  }
  double m = 0.0;
  for (double x : v) {
    m += x;
  }
  m /= static_cast<double>(n);
  double s = 0.0;
  for (double x : v) {
    s += (x - m) * (x - m);
  }
  return std::sqrt(s / static_cast<double>(n));
}

inline double median_of(std::deque<double> samples)
{
  if (samples.empty()) {
    return 0.0;
  }
  std::sort(samples.begin(), samples.end());
  const std::size_t n = samples.size();
  if (n % 2 == 1) {
    return samples[n / 2];
  }
  return 0.5 * (samples[n / 2 - 1] + samples[n / 2]);
}

class BucketMap
{
public:
  explicit BucketMap(const LockParams & p = LockParams())
  : p_(p) {}

  /// 喂一帧（单个检测，世界系）。直径先验不符直接拒（免费一致性校验）。
  UpdateResult update(double x, double y, double diameter, double t)
  {
    if (!(p_.diameter_min_m <= diameter && diameter <= p_.diameter_max_m)) {
      return UpdateResult::kRejectedDiameter;
    }
    BucketTrack * best = nullptr;
    double best_d = p_.assoc_gate_m;
    for (auto & tr : tracks_) {
      const double d = std::hypot(tr.x - x, tr.y - y);
      if (d <= best_d) {
        best_d = d;
        best = &tr;
      }
    }
    if (best == nullptr) {
      BucketTrack tr;
      tr.tid = next_id_++;
      tr.x = x;
      tr.y = y;
      tr.diameter = diameter;
      tr.first_seen = t;
      tr.last_seen = t;
      tr.positions.emplace_back(x, y);
      tracks_.push_back(tr);
      return UpdateResult::kNewTrack;
    }
    const double a = p_.ema_alpha;
    best->x = (1.0 - a) * best->x + a * x;
    best->y = (1.0 - a) * best->y + a * y;
    best->diam_samples.push_back(diameter);
    while (static_cast<int>(best->diam_samples.size()) > p_.diameter_window) {
      best->diam_samples.pop_front();
    }
    best->diameter = median_of(best->diam_samples);
    best->positions.emplace_back(best->x, best->y);
    while (best->positions.size() > 10) {
      best->positions.pop_front();
    }
    ++best->confirms;
    best->last_seen = t;
    return UpdateResult::kAssociated;
  }

  std::vector<BucketTrack> confirmed(double t) const
  {
    std::vector<BucketTrack> out;
    for (const auto & tr : tracks_) {
      if (t - tr.last_seen > p_.fresh_window_s) {
        continue;
      }
      if (tr.confirms < p_.min_confirm_frames) {
        continue;
      }
      if (jitter(tr) > p_.jitter_gate_m) {
        continue;
      }
      out.push_back(tr);
    }
    return out;
  }

  /// 独立性门控（真实生效版）：两确认航迹 间距<0.20m 或 直径差<0.025m →
  /// 判同一筒强制合并。HIT 区赛"两瓶投一点"的直接防线。返回合并次数。
  int enforce_independence(double t)
  {
    bool changed = true;
    int merges = 0;
    while (changed) {
      changed = false;
      auto trs = confirmed(t);
      for (std::size_t i = 0; i < trs.size() && !changed; ++i) {
        for (std::size_t j = i + 1; j < trs.size() && !changed; ++j) {
          const double dist = std::hypot(trs[i].x - trs[j].x, trs[i].y - trs[j].y);
          const double ddiff = std::fabs(trs[i].diameter - trs[j].diameter);
          if (dist < p_.min_spacing_m || ddiff < p_.min_diameter_diff_m) {
            const bool keep_i = trs[i].confirms >= trs[j].confirms;
            const int drop_tid = keep_i ? trs[j].tid : trs[i].tid;
            remove_by_tid(drop_tid);
            ++merges;
            changed = true;
          }
        }
      }
    }
    return merges;
  }

  /// 0/1/2 → 15/20/25cm；对不上号返回 nullopt。
  std::optional<int> diameter_class(double d) const
  {
    std::optional<int> best;
    double best_err = p_.diameter_match_tol_m;
    for (int i = 0; i < 3; ++i) {
      const double err = std::fabs(d - p_.nominal_diameters[i]);
      if (err <= best_err) {
        best = i;
        best_err = err;
      }
    }
    return best;
  }

  /// 锁定五步⑤：排名签名连续 rank_stable_s 不变才冻结目标集。
  LockResult try_lock(double t, bool degraded = false)
  {
    enforce_independence(t);
    std::vector<BucketTrack> trs;
    for (const auto & tr : confirmed(t)) {
      if (is_blacklisted(tr.x, tr.y, t)) {
        continue;                       // 已投/已弃筒绝不回锁
      }
      trs.push_back(tr);
    }
    std::vector<std::pair<BucketTrack, int>> classified, unknown;
    for (const auto & tr : trs) {
      const auto cls = diameter_class(tr.diameter);
      if (cls.has_value()) {
        classified.emplace_back(tr, *cls);
      } else {
        unknown.emplace_back(tr, -1);
      }
    }
    const int need = degraded ? p_.degraded_classified : p_.preferred_targets;
    if (static_cast<int>(classified.size()) < need) {
      return fail(std::string("classified ") + std::to_string(classified.size()) +
                  " < " + std::to_string(need));
    }
    if (!degraded) {
      unknown.clear();                  // 正式模式：未知筒不入选（宁缺勿错）
    }
    std::sort(classified.begin(), classified.end(),
      [](const auto & a, const auto & b) {
        if (a.second != b.second) {
          return a.second < b.second;   // 直径类升序（aggressive 语义）
        }
        return a.first.diameter < b.first.diameter;
      });
    std::vector<FrozenTarget> targets;
    for (const auto & tc : classified) {
      targets.push_back(make_target(tc.first, tc.second, t));
    }
    for (const auto & tc : unknown) {
      targets.push_back(make_target(tc.first, tc.second, t));
    }
    // 排名签名 = 有序 tid 序列
    if (signature_ != current_signature(targets)) {
      signature_ = current_signature(targets);
      rank_since_ = t;
      return fail("rank_changed");
    }
    if (!rank_since_.has_value() || t - *rank_since_ < p_.rank_stable_s) {
      return fail("rank_not_stable");
    }
    return LockResult{true, "locked " + std::to_string(targets.size()), targets};
  }

  void blacklist(double x, double y, double t)
  {
    released_.emplace_back(x, y, t);
  }

  bool is_blacklisted(double x, double y, double t) const
  {
    for (const auto & r : released_) {
      if (std::hypot(x - std::get<0>(r), y - std::get<1>(r)) <= p_.blacklist_radius_m &&
        t - std::get<2>(r) <= p_.blacklist_window_s)
      {
        return true;
      }
    }
    return false;
  }

  const LockParams & params() const {return p_;}
  LockParams & params() {return p_;}

  /// 诊断用：当前航迹总数（含未确认）。
  std::size_t track_count() const {return tracks_.size();}

private:
  static std::vector<int> current_signature(const std::vector<FrozenTarget> & ts)
  {
    std::vector<int> s;
    s.reserve(ts.size());
    for (const auto & t : ts) {
      s.push_back(t.tid);
    }
    return s;
  }

  static FrozenTarget make_target(const BucketTrack & tr, int cls, double t)
  {
    FrozenTarget ft;
    ft.tid = tr.tid;
    ft.frozen_x = tr.x;
    ft.frozen_y = tr.y;
    ft.frozen_diameter = tr.diameter;
    ft.diameter_class = cls;
    ft.working_x = tr.x;
    ft.working_y = tr.y;
    ft.last_vision_t = t;
    return ft;
  }

  LockResult fail(const std::string & reason) const
  {
    LockResult r;
    r.ok = false;
    r.reason = reason;
    return r;
  }

  static double jitter(const BucketTrack & tr)
  {
    if (tr.positions.size() < 2) {
      return std::numeric_limits<double>::infinity();
    }
    std::deque<double> xs, ys;
    for (const auto & pnt : tr.positions) {
      xs.push_back(pnt.first);
      ys.push_back(pnt.second);
    }
    return std::max(stddev_pop(xs), stddev_pop(ys));
  }

  void remove_by_tid(int tid)
  {
    tracks_.erase(
      std::remove_if(tracks_.begin(), tracks_.end(),
        [tid](const BucketTrack & tr) {return tr.tid == tid;}),
      tracks_.end());
  }

  LockParams p_;
  std::vector<BucketTrack> tracks_;
  std::vector<int> signature_;
  std::optional<double> rank_since_;
  int next_id_ = 0;
  std::vector<std::tuple<double, double, double>> released_;  ///< (x, y, t)
};

// ---------------------------------------------------------------------------
// ALIGN/RELEASE 段：活动估计维护 + 反盲投重捕获策略
// ---------------------------------------------------------------------------
struct TrackParams {
  double fresh_window_s = 0.5;          ///< 目标估计新鲜度（反盲投：超龄禁投）
  double ema_alpha = 0.25;
  double reacquire_pos_gate_m = 0.40;
  double reacquire_diam_gate_m = 0.06;
  double reacquire_window_s = 2.0;      ///< 丢视觉后允许等待重捕获的时长
  int max_reacquires = 1;               ///< SSOT：重捕获一次，仍丢 = 弃桶
};

struct Detection2D {
  double x = 0.0;
  double y = 0.0;
  double diameter = 0.0;
};

enum class Assess { kOk, kReacquiring, kAbandon };

class TargetTracker
{
public:
  FrozenTarget * target;                ///< 指向冻结目标（调用方保证生命周期）
  TrackParams p;

  TargetTracker(FrozenTarget * tgt, const TrackParams & params = TrackParams())
  : target(tgt), p(params) {}

  /// 每帧喂世界系检测（全部检测，本类按门限自选）。返回是否关联成功。
  bool update(const std::vector<Detection2D> & detections, double t)
  {
    const Detection2D * best = nullptr;
    double best_d = p.reacquire_pos_gate_m;
    for (const auto & d : detections) {
      if (std::fabs(d.diameter - target->frozen_diameter) > p.reacquire_diam_gate_m) {
        continue;
      }
      const double dist = std::hypot(d.x - target->working_x, d.y - target->working_y);
      if (dist <= best_d) {
        best_d = dist;
        best = &d;
      }
    }
    if (best == nullptr) {
      return false;
    }
    const double a = p.ema_alpha;
    target->working_x = (1.0 - a) * target->working_x + a * best->x;
    target->working_y = (1.0 - a) * target->working_y + a * best->y;
    target->last_vision_t = t;
    target->lost_since_valid = false;
    return true;
  }

  double vision_age(double t) const {return t - target->last_vision_t;}

  /// kOk=新鲜；kReacquiring=丢视觉、窗口内（禁投）；kAbandon=机会用尽 → 弃桶。
  Assess assess(double t)
  {
    const bool lost = vision_age(t) > p.fresh_window_s;
    if (!lost) {
      return Assess::kOk;
    }
    if (target->reacquires_used >= p.max_reacquires) {
      return Assess::kAbandon;
    }
    if (!target->lost_since_valid) {
      target->lost_since_valid = true;
      target->lost_since = t;
      return Assess::kReacquiring;
    }
    if (t - target->lost_since > p.reacquire_window_s) {
      target->reacquires_used = p.max_reacquires;   // 窗口耗尽 = 机会用掉
      return Assess::kAbandon;
    }
    return Assess::kReacquiring;
  }

  void on_reacquired() {++target->reacquires_used;}
};

// ---------------------------------------------------------------------------
// RELEASE 段：八门控 + 弹道前移 + 单发舵机
// ---------------------------------------------------------------------------
struct GateParams {
  double max_horizontal_error_m = 0.10;
  double max_vertical_error_m = 0.10;
  double max_hspeed_m_s = 0.08;
  double max_vspeed_m_s = 0.05;
  double max_tilt_deg = 5.0;
  double max_yaw_err_deg = 5.0;
  double stability_s = 0.8;             ///< 连续无违约稳定
  double hold_s = 1.5;                  ///< 达稳定后再保持（合计连续 2.3s）
  double timeout_s = 6.0;               ///< 进入 RELEASE 起 6s 未凑齐 → 弃桶
  double max_target_age_s = 0.5;        ///< 反盲投：目标估计超龄禁投
  double max_lead_m = 0.15;             ///< 弹道前移 sanity 上限（悬停 ≈0）
  double release_height_m = 1.8;        ///< 释放高度（SSOT 对准高度）
};

struct GateSample {
  double horiz_err_m = 0.0;
  double vert_err_m = 0.0;
  double hspeed_m_s = 0.0;
  double vspeed_m_s = 0.0;
  double tilt_deg = 0.0;
  double yaw_err_deg = 0.0;
  double target_age_s = 0.0;
  double vx_m_s = 0.0;                  ///< 世界系速度（弹道前移 sanity 用）
  double vy_m_s = 0.0;
  double t = 0.0;
};

inline std::pair<double, double> ballistic_lead(double vx, double vy, double height_m,
  double g = kG)
{
  const double t_fall = std::sqrt(2.0 * std::max(0.0, height_m) / g);
  return {vx * t_fall, vy * t_fall};
}

enum class GateStatus { kIdle, kHolding, kFire, kFired, kAbort };

struct GateVerdict {
  GateStatus status = GateStatus::kIdle;
  std::string reason;
};

class ReleaseGate
{
public:
  explicit ReleaseGate(const GateParams & p = GateParams())
  : p_(p) {}

  void reset()
  {
    pass_started_.reset();
    entered_t_.reset();
    fired_ = false;
    aborted_ = false;
    abort_reason_.clear();
  }

  /// 每拍一次。kFire 只出现一次，此后恒 kFired（幂等）；违约清零计时器。
  GateVerdict feed(const GateSample & s)
  {
    if (fired_) {
      return {GateStatus::kFired, "already_fired"};
    }
    if (aborted_) {
      return {GateStatus::kAbort, abort_reason_};   // 保留原始弃桶原因，排障用
    }
    if (!entered_t_.has_value()) {
      entered_t_ = s.t;
    }
    const auto inst = instant(s);
    if (!inst.first) {
      pass_started_.reset();
      if (s.t - *entered_t_ > p_.timeout_s) {
        aborted_ = true;
        abort_reason_ = "timeout_last_reason=" + inst.second;
        return {GateStatus::kAbort, abort_reason_};
      }
      return {GateStatus::kIdle, inst.second};
    }
    if (!pass_started_.has_value()) {
      pass_started_ = s.t;
    }
    const double held = s.t - *pass_started_;
    if (held >= p_.stability_s + p_.hold_s) {
      fired_ = true;
      return {GateStatus::kFire, "gates_satisfied"};
    }
    if (s.t - *entered_t_ > p_.timeout_s) {
      aborted_ = true;
      abort_reason_ = "timeout";
      return {GateStatus::kAbort, abort_reason_};
    }
    return {GateStatus::kHolding, "held=" + std::to_string(held) + "s"};
  }

  bool fired() const {return fired_;}
  bool aborted() const {return aborted_;}
  const GateParams & params() const {return p_;}
  GateParams & params() {return p_;}

private:
  std::pair<bool, std::string> instant(const GateSample & s) const
  {
    const double vals[] = {s.horiz_err_m, s.vert_err_m, s.hspeed_m_s, s.vspeed_m_s,
      s.tilt_deg, s.yaw_err_deg, s.target_age_s, s.t};
    for (double v : vals) {
      if (!std::isfinite(v)) {
        return {false, "non_finite"};
      }
    }
    if (s.target_age_s > p_.max_target_age_s) {
      return {false, "target_stale"};           // 反盲投核心防线
    }
    if (s.horiz_err_m > p_.max_horizontal_error_m) {
      return {false, "horizontal_error"};
    }
    if (std::fabs(s.vert_err_m) > p_.max_vertical_error_m) {
      return {false, "vertical_error"};
    }
    if (s.hspeed_m_s > p_.max_hspeed_m_s) {
      return {false, "horizontal_speed"};
    }
    if (std::fabs(s.vspeed_m_s) > p_.max_vspeed_m_s) {
      return {false, "vertical_speed"};
    }
    if (s.tilt_deg > p_.max_tilt_deg) {
      return {false, "tilt"};
    }
    if (s.yaw_err_deg > p_.max_yaw_err_deg) {
      return {false, "yaw_error"};
    }
    const auto lead = ballistic_lead(s.vx_m_s, s.vy_m_s, p_.release_height_m);
    if (std::hypot(lead.first, lead.second) > p_.max_lead_m) {
      return {false, "lead_excess"};            // sanity：速度门限失守的第二道闸
    }
    return {true, "ok"};
  }

  GateParams p_;
  std::optional<double> pass_started_;
  std::optional<double> entered_t_;
  bool fired_ = false;
  bool aborted_ = false;
  std::string abort_reason_;
};

enum class SeqResult { kNone, kStow };

class DropSequencer
{
public:
  static constexpr double kHoldS = 0.7;   ///< SSOT §5.4：释放保持 0.7s 后回仓

  /// 单发保护：非 IDLE 态调用直接拒绝——双发防线是硬性的。
  bool fire(double t)
  {
    if (phase_ != Phase::kIdle) {
      return false;
    }
    phase_ = Phase::kReleased;
    fire_t_ = t;
    return true;
  }

  /// 返回 kStow（恰好一次）表示该回仓。
  SeqResult tick(double t)
  {
    if (phase_ == Phase::kReleased && t - fire_t_ >= kHoldS) {
      phase_ = Phase::kStowed;
      return SeqResult::kStow;
    }
    return SeqResult::kNone;
  }

  bool is_idle() const {return phase_ == Phase::kIdle;}

private:
  enum class Phase { kIdle, kReleased, kStowed };
  Phase phase_ = Phase::kIdle;
  double fire_t_ = 0.0;
};

// ---------------------------------------------------------------------------
// 瞄准点解算（冻结 + 标定偏置 + 弹道前移）
// ---------------------------------------------------------------------------
struct AimResult {
  double ax = 0.0;
  double ay = 0.0;
  double lead_x = 0.0;
  double lead_y = 0.0;
};

inline AimResult aim_point(const FrozenTarget & frozen,
  double offset_body_x, double offset_body_y,
  double mission_yaw, double vx, double vy, const GateParams & p)
{
  const double c = std::cos(mission_yaw);
  const double s = std::sin(mission_yaw);
  const double ox = c * offset_body_x - s * offset_body_y;
  const double oy = s * offset_body_x + c * offset_body_y;
  auto lead = ballistic_lead(vx, vy, p.release_height_m);
  const double norm = std::hypot(lead.first, lead.second);
  if (norm > p.max_lead_m && norm > 0.0) {
    lead.first *= p.max_lead_m / norm;
    lead.second *= p.max_lead_m / norm;
  }
  return AimResult{frozen.working_x + ox + lead.first,
    frozen.working_y + oy + lead.second, lead.first, lead.second};
}

/// 冻结集排序：conservative=直径类降序（先大筒保底）；aggressive=升序；未知排最后。
inline std::vector<FrozenTarget> sorted_targets(std::vector<FrozenTarget> targets,
  const std::string & drop_order)
{
  std::vector<FrozenTarget> known, unknown;
  for (auto & t : targets) {
    (t.diameter_class >= 0 ? known : unknown).push_back(t);
  }
  std::sort(known.begin(), known.end(),
    [&drop_order](const FrozenTarget & a, const FrozenTarget & b) {
      if (a.diameter_class != b.diameter_class) {
        return drop_order == "conservative" ?
               a.diameter_class > b.diameter_class : a.diameter_class < b.diameter_class;
      }
      return a.frozen_diameter > b.frozen_diameter;
    });
  known.insert(known.end(), unknown.begin(), unknown.end());
  return known;
}

}  // namespace cuadc_drop
