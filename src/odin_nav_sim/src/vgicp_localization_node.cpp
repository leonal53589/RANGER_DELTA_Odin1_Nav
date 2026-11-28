#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "angles/angles.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "vgicp_4dof_matcher.hpp"

namespace odin_nav_sim
{
namespace
{

struct OdomSample
{
  int64_t stamp_ns{};
  double x{};
  double y{};
  double yaw{};
  double linear_speed{};
  double yaw_rate{};
  double distance_traveled{};
  double angle_traveled{};
};

struct SE2
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

SE2 composeSE2(const SE2 & a, const SE2 & b)
{
  const double c = std::cos(a.yaw);
  const double s = std::sin(a.yaw);
  return {
    a.x + c * b.x - s * b.y,
    a.y + s * b.x + c * b.y,
    std::remainder(a.yaw + b.yaw, 2.0 * M_PI),
  };
}

SE2 inverseSE2(const SE2 & a)
{
  const double c = std::cos(a.yaw);
  const double s = std::sin(a.yaw);
  return {
    -(c * a.x + s * a.y),
    s * a.x - c * a.y,
    std::remainder(-a.yaw, 2.0 * M_PI),
  };
}

SE2 relativeSE2(const SE2 & from, const SE2 & to)
{
  return composeSE2(inverseSE2(from), to);
}

double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion tq(q.x, q.y, q.z, q.w);
  double r, p, y;
  tf2::Matrix3x3(tq).getRPY(r, p, y);
  return y;
}

std::optional<size_t> floatFieldOffset(
  const sensor_msgs::msg::PointCloud2 & msg, const std::string & name)
{
  for (const auto & f : msg.fields) {
    if (f.name == name) {
      if (f.datatype != sensor_msgs::msg::PointField::FLOAT32 || f.count != 1 ||
        f.offset + sizeof(float) > msg.point_step)
      {
        return std::nullopt;
      }
      return static_cast<size_t>(f.offset);
    }
  }
  return std::nullopt;
}

class VgicpLocalizationNode : public rclcpp::Node
{
public:
  VgicpLocalizationNode()
  : Node("vgicp_localization")
  {
    const std::string reference_pcd = declare_parameter<std::string>("reference_pcd", "");
    if (reference_pcd.empty()) {
      throw std::invalid_argument("reference_pcd parameter is required");
    }

    MountExtrinsics mount;
    mount.tx = declare_parameter<double>("mount_x", 0.1754);
    mount.ty = declare_parameter<double>("mount_y", 0.0025);
    mount.tz = declare_parameter<double>("mount_z", 1.1241);
    mount.roll = declare_parameter<double>("mount_roll", -0.00085);
    mount.pitch = declare_parameter<double>("mount_pitch", 0.01325);
    mount.yaw = declare_parameter<double>("mount_yaw", -0.02631);

    MatcherConfig cfg;
    cfg.num_threads = std::max(1, static_cast<int>(declare_parameter<int>("num_threads", 2)));
    cfg.weak_damp = declare_parameter<double>("weak_damp", 300.0);
    cfg.weak_eigen_ratio = declare_parameter<double>("weak_eigen_ratio", 0.08);
    cfg.max_innovation_xy = declare_parameter<double>("locked_innovation_xy", 0.25);
    cfg.max_innovation_yaw = declare_parameter<double>("locked_innovation_yaw", 0.21);

    cold_innovation_xy_ = declare_parameter<double>("cold_innovation_xy", 0.50);
    cold_innovation_yaw_ = declare_parameter<double>("cold_innovation_yaw", 0.45);
    moving_fusion_alpha_ = declare_parameter<double>("moving_fusion_alpha", 0.22);
    moving_max_step_xy_ = declare_parameter<double>("moving_max_step_xy", 0.015);
    moving_max_step_yaw_ = declare_parameter<double>("moving_max_step_yaw", 0.025);
    stationary_v_max_ = declare_parameter<double>("stationary_v_max", 0.015);
    stationary_w_max_ = declare_parameter<double>("stationary_w_max", 0.030);

    const double odin_x = declare_parameter<double>("odin_map_x", 0.0);
    const double odin_y = declare_parameter<double>("odin_map_y", 0.0);
    const double odin_yaw = declare_parameter<double>("odin_map_yaw", 0.0);
    map_from_odin_map_ = SE2{odin_x, odin_y, odin_yaw};

    use_odin_seed_ = declare_parameter<bool>("use_odin_seed", true);
    review_ambiguity_ = declare_parameter<bool>("review_ambiguity", true);
    tf_stamp_lead_ = declare_parameter<double>("tf_stamp_lead", 0.30);
    max_pose_age_ = declare_parameter<double>("max_pose_age", 0.50);
    max_fix_timeout_ = declare_parameter<double>("max_fix_timeout", 8.0);
    max_fix_distance_ = declare_parameter<double>("max_fix_distance", 0.8);
    max_fix_turn_ = declare_parameter<double>("max_fix_turn", 0.50);
    stationary_recovery_ = declare_parameter<bool>("stationary_recovery", true);
    recovery_min_height_ = declare_parameter<double>("recovery_min_height", 0.70);
    if (!std::isfinite(max_fix_timeout_) || max_fix_timeout_ <= 0.0 ||
      !std::isfinite(max_fix_distance_) || max_fix_distance_ <= 0.0 ||
      !std::isfinite(max_fix_turn_) || max_fix_turn_ <= 0.0 ||
      !std::isfinite(recovery_min_height_) ||
      recovery_min_height_ < cfg.height_min || recovery_min_height_ >= cfg.height_max)
    {
      throw std::invalid_argument("Invalid VGICP odometry budget or stationary recovery height");
    }
    scan_stride_ = std::max(1, static_cast<int>(declare_parameter<int>("scan_stride", 5)));
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    lidar_frame_ = declare_parameter<std::string>("lidar_frame", "lidar");

    const std::string cloud_topic = declare_parameter<std::string>("cloud_topic", "/odin1/cloud_raw");
    const std::string odom_topic = declare_parameter<std::string>("odom_topic", "/odom");
    const std::string odin_tf_topic = declare_parameter<std::string>("odin_tf_topic", "/odin1/tf");
    const std::string odin_tf_static_topic = declare_parameter<std::string>(
      "odin_tf_static_topic", "/odin1/tf_static");

    RCLCPP_INFO(get_logger(), "Loading GLIM reference PCD: %s", reference_pcd.c_str());
    const auto map_pts = loadPcdXyz(reference_pcd);
    matcher_ = std::make_unique<Vgicp4DofMatcher>(map_pts, mount, cfg);
    RCLCPP_INFO(
      get_logger(), "Built dual-voxel VGICP target (%zu preprocessed points)",
      matcher_->targetSize());

    io_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    scan_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions io_opts;
    io_opts.callback_group = io_group_;
    rclcpp::SubscriptionOptions scan_opts;
    scan_opts.callback_group = scan_group_;

    odin_tf_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    odin_tf_->setUsingDedicatedThread(true);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/vgicp_localization/pose", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/vgicp_localization/status", 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS().keep_last(100),
      std::bind(&VgicpLocalizationNode::onOdom, this, std::placeholders::_1), io_opts);

    odin_tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      odin_tf_topic, 100,
      std::bind(&VgicpLocalizationNode::onOdinTf, this, std::placeholders::_1), io_opts);

    odin_tf_static_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      odin_tf_static_topic, rclcpp::QoS(100).transient_local(),
      std::bind(&VgicpLocalizationNode::onOdinTfStatic, this, std::placeholders::_1), io_opts);

    initialpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10,
      std::bind(&VgicpLocalizationNode::onInitialPose, this, std::placeholders::_1), io_opts);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&VgicpLocalizationNode::onCloud, this, std::placeholders::_1), scan_opts);

    publish_timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&VgicpLocalizationNode::onPublishTimer, this), io_group_);
  }

private:
  void onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    if (msg->header.frame_id != odom_frame_ || msg->child_frame_id != base_frame_) {
      return;
    }
    const auto & p = msg->pose.pose.position;
    const auto & q = msg->pose.pose.orientation;
    const auto & tw = msg->twist.twist;
    const double yaw = yawFromQuat(q);
    const double v = std::hypot(tw.linear.x, tw.linear.y);
    const double w = tw.angular.z;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(yaw) ||
      !std::isfinite(v) || !std::isfinite(w))
    {
      return;
    }
    const int64_t t_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
    std::lock_guard<std::mutex> lock(state_mutex_);
    double distance_traveled = 0.0;
    double angle_traveled = 0.0;
    if (!odom_history_.empty()) {
      const auto & prev = odom_history_.back();
      if (t_ns <= prev.stamp_ns) {
        return;
      }
      const double step_xy = std::hypot(p.x - prev.x, p.y - prev.y);
      const double step_yaw = std::abs(std::remainder(yaw - prev.yaw, 2.0 * M_PI));
      distance_traveled = prev.distance_traveled + step_xy;
      angle_traveled = prev.angle_traveled + step_yaw;
      if (step_xy > 0.30 || step_yaw > 0.35) {
        RCLCPP_WARN(
          get_logger(), "Detected discontinuous /odom step (%.2fm, %.2frad); resetting VGICP lock",
          step_xy, step_yaw);
        has_lock_ = false;
        ++localization_epoch_;
        odom_history_.clear();
      }
    }
    odom_history_.push_back({t_ns, p.x, p.y, yaw, v, w, distance_traveled, angle_traveled});
    while (odom_history_.size() > 300 ||
      (!odom_history_.empty() && t_ns - odom_history_.front().stamp_ns > 5'000'000'000LL))
    {
      odom_history_.pop_front();
    }
  }

  void onOdinTf(const tf2_msgs::msg::TFMessage::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(odin_tf_mutex_);
    for (const auto & tf : msg->transforms) {
      odin_tf_->setTransform(tf, "odin_driver");
    }
  }

  void onOdinTfStatic(const tf2_msgs::msg::TFMessage::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(odin_tf_mutex_);
    for (const auto & tf : msg->transforms) {
      odin_tf_->setTransform(tf, "odin_driver", true);
    }
  }

  void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
  {
    if (!msg->header.frame_id.empty() && msg->header.frame_id != global_frame_) {
      RCLCPP_WARN(
        get_logger(), "Ignoring /initialpose in unexpected frame '%s'",
        msg->header.frame_id.c_str());
      return;
    }
    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;
    const double yaw = yawFromQuat(msg->pose.pose.orientation);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
      return;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    pending_initial_base_ = SE2{x, y, yaw};
    has_lock_ = false;
    ++localization_epoch_;
    RCLCPP_INFO(
      get_logger(), "Received /initialpose seed (%.3f, %.3f, %.1f deg) — relocking VGICP",
      x, y, yaw * 180.0 / M_PI);
  }

  std::optional<SE2> interpolateOdomLocked(int64_t target_ns, int64_t * latest_ns = nullptr) const
  {
    if (odom_history_.empty()) {
      return std::nullopt;
    }
    if (latest_ns) {
      *latest_ns = odom_history_.back().stamp_ns;
    }
    if (target_ns < odom_history_.front().stamp_ns - 50'000'000LL) {
      return std::nullopt;
    }
    if (target_ns <= odom_history_.front().stamp_ns) {
      const auto & s = odom_history_.front();
      return SE2{s.x, s.y, s.yaw};
    }
    if (target_ns >= odom_history_.back().stamp_ns) {
      const auto & last = odom_history_.back();
      const double dt = static_cast<double>(target_ns - last.stamp_ns) * 1e-9;
      if (dt > 0.10) {
        return std::nullopt;
      }
      // Small zero-order / yaw-rate extrapolation for tail points of active scan.
      const double dyaw = last.yaw_rate * dt;
      return SE2{last.x, last.y, std::remainder(last.yaw + dyaw, 2.0 * M_PI)};
    }
    auto it = std::upper_bound(
      odom_history_.begin(), odom_history_.end(), target_ns,
      [](int64_t val, const OdomSample & s) {return val < s.stamp_ns;});
    const auto & b = *it;
    const auto & a = *std::prev(it);
    const double span = static_cast<double>(b.stamp_ns - a.stamp_ns);
    const double u = span > 1.0 ? static_cast<double>(target_ns - a.stamp_ns) / span : 0.0;
    const double dyaw = std::remainder(b.yaw - a.yaw, 2.0 * M_PI);
    return SE2{
      a.x + u * (b.x - a.x),
      a.y + u * (b.y - a.y),
      std::remainder(a.yaw + u * dyaw, 2.0 * M_PI),
    };
  }

  bool isStationaryLocked(int64_t scan_ns) const
  {
    const int64_t start = scan_ns - 120'000'000LL;
    const int64_t end = scan_ns + 95'000'000LL;
    if (odom_history_.size() < 3 ||
      odom_history_.front().stamp_ns > start ||
      odom_history_.back().stamp_ns < end)
    {
      return false;
    }
    int count = 0;
    int64_t prev_t = 0;
    for (const auto & s : odom_history_) {
      if (s.stamp_ns < start - 40'000'000LL || s.stamp_ns > end + 40'000'000LL) {
        continue;
      }
      if (count > 0 && (s.stamp_ns - prev_t) > 80'000'000LL) {
        return false;
      }
      if (s.linear_speed > stationary_v_max_ || std::abs(s.yaw_rate) > stationary_w_max_) {
        return false;
      }
      prev_t = s.stamp_ns;
      ++count;
    }
    return count >= 3;
  }

  std::optional<Eigen::Vector3d> odinRadarSeed(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped odin_tf;
    {
      std::lock_guard<std::mutex> lock(odin_tf_mutex_);
      try {
        odin_tf = odin_tf_->lookupTransform("odin_map", lidar_frame_, stamp);
      } catch (const tf2::TransformException &) {
        try {
          odin_tf = odin_tf_->lookupTransform("odin_map", lidar_frame_, tf2::TimePointZero);
        } catch (const tf2::TransformException &) {
          return std::nullopt;
        }
      }
    }
    const int64_t odin_ns = rclcpp::Time(odin_tf.header.stamp).nanoseconds();
    if (std::abs(stamp.nanoseconds() - odin_ns) > 500'000'000LL) {
      return std::nullopt;
    }
    const auto & t = odin_tf.transform.translation;
    const double lidar_yaw = yawFromQuat(odin_tf.transform.rotation);
    const SE2 map_lidar = composeSE2(map_from_odin_map_, SE2{t.x, t.y, lidar_yaw});
    // Convert full sensor orientation to leveled radar frame (remove mount yaw, then re-add).
    const SE2 map_base = composeSE2(
      map_lidar,
      inverseSE2(SE2{matcher_->mount().tx, matcher_->mount().ty, matcher_->mount().yaw}));
    return matcher_->mount().baseToRadar({map_base.x, map_base.y, map_base.yaw});
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    const int64_t scan_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
    const int64_t now_ns = get_clock()->now().nanoseconds();
    if (msg->header.frame_id != lidar_frame_ || scan_ns <= 0 ||
      now_ns - scan_ns < -50'000'000LL || now_ns - scan_ns > 450'000'000LL)
    {
      publishStatus(scan_ns, false, false, "frame_or_scan_time", {});
      return;
    }
    if (msg->is_bigendian || msg->height == 0 || msg->width == 0 || msg->point_step == 0 ||
      static_cast<size_t>(msg->width) * msg->point_step != msg->row_step ||
      msg->data.size() != static_cast<size_t>(msg->row_step) * msg->height)
    {
      publishStatus(scan_ns, false, false, "invalid_cloud_layout", {});
      return;
    }
    const auto x_off = floatFieldOffset(*msg, "x");
    const auto y_off = floatFieldOffset(*msg, "y");
    const auto z_off = floatFieldOffset(*msg, "z");
    if (!x_off || !y_off || !z_off) {
      publishStatus(scan_ns, false, false, "missing_xyz_fields", {});
      return;
    }
    const auto ot_off = floatFieldOffset(*msg, "offset_time");

    // Extract strided raw points first (fast, lock-free).
    std::vector<RawLidarPoint> raw_points;
    const size_t total_pts = msg->data.size() / msg->point_step;
    raw_points.reserve(total_pts / static_cast<size_t>(scan_stride_) + 1);
    const uint8_t * data = msg->data.data();
    float max_offset = 0.0f;
    for (size_t idx = 0; idx < total_pts; idx += static_cast<size_t>(scan_stride_)) {
      const uint8_t * ptr = data + idx * msg->point_step;
      RawLidarPoint rp;
      std::memcpy(&rp.x, ptr + *x_off, sizeof(float));
      std::memcpy(&rp.y, ptr + *y_off, sizeof(float));
      std::memcpy(&rp.z, ptr + *z_off, sizeof(float));
      if (ot_off) {
        std::memcpy(&rp.offset_s, ptr + *ot_off, sizeof(float));
      }
      if (std::isfinite(rp.x) && std::isfinite(rp.y) && std::isfinite(rp.z) &&
        rp.x > 0.35f)
      {
        if (std::isfinite(rp.offset_s) && rp.offset_s >= 0.0f && rp.offset_s < 0.25f) {
          max_offset = std::max(max_offset, rp.offset_s);
        } else {
          rp.offset_s = 0.0f;
        }
        raw_points.push_back(rp);
      }
    }

    // Read odom seed & motion window.
    bool stationary = false;
    bool has_lock_snapshot = false;
    int64_t fix_age_ns = 0;
    uint64_t epoch = 0;
    double fix_distance_origin = 0.0;
    double fix_turn_origin = 0.0;
    std::optional<SE2> odom_at_scan;
    SE2 predicted_base;
    std::vector<MotionDeltaSE2> motion_track;
    std::optional<Eigen::Vector3d> seed_radar;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      odom_at_scan = interpolateOdomLocked(scan_ns);
      if (!odom_at_scan) {
        publishStatus(scan_ns, false, false, "missing_odom", {});
        return;
      }

      epoch = localization_epoch_;
      const auto next_sample = std::upper_bound(
        odom_history_.begin(), odom_history_.end(), scan_ns,
        [](int64_t t, const OdomSample & sample) {return t < sample.stamp_ns;});
      const auto & earlier = next_sample == odom_history_.begin() ?
        odom_history_.front() : *std::prev(next_sample);
      fix_distance_origin = earlier.distance_traveled;
      fix_turn_origin = earlier.angle_traveled;
      has_lock_snapshot = has_lock_;
      fix_age_ns = has_lock_ ? (scan_ns - last_fix_ns_) : 0;
      stationary = isStationaryLocked(scan_ns);
      if (!stationary && ot_off && max_offset > 1e-4f) {
        for (int k = 0; k <= 10; ++k) {
          const double dt = max_offset * (static_cast<double>(k) / 10.0);
          const auto o_dt = interpolateOdomLocked(
            scan_ns + static_cast<int64_t>(std::llround(dt * 1e9)));
          if (o_dt) {
            const SE2 rel = relativeSE2(*odom_at_scan, *o_dt);
            motion_track.push_back({dt, rel.x, rel.y, rel.yaw});
          }
        }
      }

      if (pending_initial_base_) {
        seed_radar = matcher_->mount().baseToRadar(
          {pending_initial_base_->x, pending_initial_base_->y, pending_initial_base_->yaw});
        predicted_base = *pending_initial_base_;
      } else if (has_lock_) {
        predicted_base = composeSE2(map_from_odom_, *odom_at_scan);
        seed_radar = matcher_->mount().baseToRadar(
          {predicted_base.x, predicted_base.y, predicted_base.yaw});
      }
    }

    if (!seed_radar && use_odin_seed_) {
      seed_radar = odinRadarSeed(rclcpp::Time(msg->header.stamp));
      if (seed_radar) {
        const Eigen::Vector3d b = matcher_->mount().radarToBase(*seed_radar);
        predicted_base = {b.x(), b.y(), b.z()};
      }
    }
    if (!seed_radar) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "VGICP waiting for a fresh ODIN or /initialpose seed");
      publishStatus(scan_ns, false, stationary, "waiting_for_seed", {});
      return;
    }

    const auto & cfg = matcher_->config();
    const auto ground_pts = deskewAndGroundFilter(
      raw_points, matcher_->mount(), motion_track,
      cfg.ground_voxel, cfg.height_min, cfg.height_max);

    const double max_xy = has_lock_snapshot ? -1.0 : cold_innovation_xy_;
    const double max_yaw = has_lock_snapshot ? -1.0 : cold_innovation_yaw_;
    const double yaw_offset_deg = (has_lock_snapshot && !stationary) ? 2.5 : -1.0;
    MatchResult fit = matcher_->alignGroundScan(
      ground_pts, *seed_radar, max_xy, max_yaw, yaw_offset_deg,
      stationary && (review_ambiguity_ || stationary_recovery_));

    // If locked tracking has failed for >4 s (e.g. wheel slip or sharp turn), try ODIN fallback seed.
    if (!fit.accepted && has_lock_snapshot && fix_age_ns > 4'000'000'000LL && use_odin_seed_) {
      if (const auto recovery_seed = odinRadarSeed(rclcpp::Time(msg->header.stamp))) {
        const MatchResult rec = matcher_->alignGroundScan(
          ground_pts, *recovery_seed, cold_innovation_xy_, cold_innovation_yaw_);
        const Eigen::Vector3d prediction(predicted_base.x, predicted_base.y, predicted_base.yaw);
        // A moving reacquisition must be near wheel prediction. Preserve slew limiting.
        if (rec.accepted && (stationary || VgicpRecoveryWindow::near(
            rec.base_xy_yaw, prediction, cfg.max_innovation_xy, cfg.max_innovation_yaw)))
        {
          fit = rec;
          seed_radar = recovery_seed;
          RCLCPP_WARN(
            get_logger(), "VGICP recovered lock via ODIN seed at (%.3f, %.3f, %.1f deg)",
            fit.base_xy_yaw.x(), fit.base_xy_yaw.y(), fit.base_xy_yaw.z() * 180.0 / M_PI);
        }
      }
    }

    std::optional<MatchResult> recovery;
    const Eigen::Vector3d wheel_pose(odom_at_scan->x, odom_at_scan->y, odom_at_scan->yaw);
    if (recovery_epoch_ != epoch) {
      recovery_window_.reset();
      recovery_epoch_ = epoch;
    }
    if (stationary_recovery_ && stationary && isVgicpRecoveryInput(fit.reason)) {
      if (recovery_window_.add(scan_ns, wheel_pose, *seed_radar, ground_pts, fit.review.candidates)) {
        const auto start = std::chrono::steady_clock::now();
        recovery = matcher_->recoverStationary(
          recovery_window_, *seed_radar, has_lock_snapshot ? cfg.max_innovation_xy : max_xy,
          has_lock_snapshot ? cfg.max_innovation_yaw : max_yaw,
          static_cast<float>(recovery_min_height_));
        recovery_window_.reset();
        fit.solve_ms += std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - start).count();
        if (recovery->accepted) {
          recovery->solve_ms = fit.solve_ms;
          fit = *recovery;
        }
        RCLCPP_INFO(
          get_logger(), "VGICP stationary recovery: %s (cells=%zu, margin=%.2f mm)",
          recovery->reason.c_str(), recovery->review.supporting_cells,
          1000.0 * recovery->review.margin_without_best_cell_m);
      }
    } else {
      recovery_window_.reset();
    }

    if (!fit.accepted && (!has_lock_snapshot || fix_age_ns > 2'000'000'000LL)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "VGICP %s rejected (%s, stationary=%d, ov=%.3f, inl=%.3f, geom=%.3f, "
        "spread=%.2fcm, pts=%zu, review=%s)",
        has_lock_snapshot ? "tracking scan" : "initial lock",
        fit.reason.c_str(), static_cast<int>(stationary), fit.overlap_8cm, fit.inlier_ratio,
        fit.geometry_singular_ratio, fit.seed_spread_m * 100.0, ground_pts.size(),
        fit.review.reason.c_str());
    }

    if (fit.accepted) {
      const SE2 matched_base{fit.base_xy_yaw.x(), fit.base_xy_yaw.y(), fit.base_xy_yaw.z()};
      SE2 fused_base = matched_base;
      SE2 emit_alignment;
      int64_t emit_base_ns = scan_ns;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (epoch != localization_epoch_ || scan_ns <= last_fix_ns_) {
          publishStatus(scan_ns, false, false, "superseded_scan", {});
          return;
        }
        if (recovery && recovery->accepted) {
          const auto & latest = odom_history_.back();
          if (!VgicpRecoveryWindow::near(
              {latest.x, latest.y, latest.yaw}, wheel_pose, 0.008, 0.01) ||
            latest.linear_speed > stationary_v_max_ || std::abs(latest.yaw_rate) > stationary_w_max_ ||
            get_clock()->now().nanoseconds() - latest.stamp_ns >
              static_cast<int64_t>(max_pose_age_ * 1e9))
          {
            publishStatus(scan_ns, false, false, "recovery_vehicle_moved", {});
            return;
          }
        }
        if (!stationary && has_lock_ && has_lock_snapshot) {
          // Slew-limited, geometry-weighted update while driving so degenerate corridors
          // do not inject longitudinal map->odom oscillations into MPPI and costmaps.
          const double geom_scale = std::clamp(
            (fit.geometry_singular_ratio - 0.02) / 0.10, 0.20, 1.0);
          const double eff_alpha = moving_fusion_alpha_ * geom_scale;
          const double eff_max_step_xy = moving_max_step_xy_ * std::max(0.25, geom_scale);
          const double err_x = matched_base.x - predicted_base.x;
          const double err_y = matched_base.y - predicted_base.y;
          const double err_xy = std::hypot(err_x, err_y);
          const double err_yaw = std::remainder(matched_base.yaw - predicted_base.yaw, 2.0 * M_PI);
          const double step_xy = std::min(eff_max_step_xy, eff_alpha * err_xy);
          const double xy_scale = err_xy > 1e-9 ? step_xy / err_xy : 0.0;
          const double step_yaw = std::clamp(
            eff_alpha * err_yaw, -moving_max_step_yaw_, moving_max_step_yaw_);
          fused_base = {
            predicted_base.x + err_x * xy_scale,
            predicted_base.y + err_y * xy_scale,
            std::remainder(predicted_base.yaw + step_yaw, 2.0 * M_PI),
          };
        }
        map_from_odom_ = composeSE2(fused_base, inverseSE2(*odom_at_scan));
        emit_alignment = map_from_odom_;
        if (!odom_history_.empty()) {
          emit_base_ns = std::max(emit_base_ns, odom_history_.back().stamp_ns);
        }
        fix_distance_origin_ = fix_distance_origin;
        fix_turn_origin_ = fix_turn_origin;
        last_fix_ns_ = scan_ns;
        if (!has_lock_) {
          RCLCPP_INFO(
            get_logger(),
            "VGICP locked in %s at (%.3f, %.3f, %.1f deg), overlap=%.3f, spread=%.2f cm",
            global_frame_.c_str(), fused_base.x, fused_base.y, fused_base.yaw * 180.0 / M_PI,
            fit.overlap_8cm, fit.seed_spread_m * 100.0);
        }
        has_lock_ = true;
        pending_initial_base_.reset();
      }

      publishBasePose(msg->header.stamp, fused_base);
      broadcastMapToOdom(emit_base_ns, emit_alignment);
    }

    publishStatus(scan_ns, fit.accepted, stationary, fit.reason, fit, seed_radar,
      recovery ? recovery->reason.c_str() : nullptr);
  }

  void onPublishTimer()
  {
    int64_t odom_ns = 0;
    SE2 alignment;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!has_lock_ || odom_history_.empty()) {
        return;
      }
      odom_ns = odom_history_.back().stamp_ns;
      const int64_t now_ns = get_clock()->now().nanoseconds();
      const double odom_age = static_cast<double>(now_ns - odom_ns) * 1e-9;
      const double fix_age = static_cast<double>(now_ns - last_fix_ns_) * 1e-9;
      if (odom_age < -0.05 || odom_age > max_pose_age_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Waiting for fresh wheel odometry (age=%.3f s)", odom_age);
        return;
      }
      const auto & wheel = odom_history_.back();
      const double distance = wheel.distance_traveled - fix_distance_origin_;
      const double turn = wheel.angle_traveled - fix_turn_origin_;
      if (!withinVgicpOdomBudget(
          fix_age, distance, turn, max_fix_timeout_, max_fix_distance_, max_fix_turn_))
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "VGICP wheel-only budget exhausted (%.2f s, %.2f m, %.2f rad); pausing map -> odom TF",
          fix_age, distance, turn);
        return;
      }
      alignment = map_from_odom_;
    }
    broadcastMapToOdom(odom_ns, alignment);
  }

  void broadcastMapToOdom(int64_t base_stamp_ns, const SE2 & alignment)
  {
    const int64_t lead_ns = static_cast<int64_t>(std::llround(tf_stamp_lead_ * 1e9));
    const int64_t emit_ns = base_stamp_ns + lead_ns;
    {
      std::lock_guard<std::mutex> lock(tf_emit_mutex_);
      if (emit_ns <= last_tf_emit_ns_) {
        return;
      }
      last_tf_emit_ns_ = emit_ns;
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = rclcpp::Time(emit_ns, get_clock()->get_clock_type());
    tf_msg.header.frame_id = global_frame_;
    tf_msg.child_frame_id = odom_frame_;
    tf_msg.transform.translation.x = alignment.x;
    tf_msg.transform.translation.y = alignment.y;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation.z = std::sin(alignment.yaw * 0.5);
    tf_msg.transform.rotation.w = std::cos(alignment.yaw * 0.5);
    tf_broadcaster_->sendTransform(tf_msg);
  }

  void publishBasePose(const builtin_interfaces::msg::Time & stamp, const SE2 & base_pose)
  {
    geometry_msgs::msg::PoseStamped out;
    out.header.stamp = stamp;
    out.header.frame_id = global_frame_;
    out.pose.position.x = base_pose.x;
    out.pose.position.y = base_pose.y;
    out.pose.orientation.z = std::sin(base_pose.yaw * 0.5);
    out.pose.orientation.w = std::cos(base_pose.yaw * 0.5);
    pose_pub_->publish(out);
  }

  void publishStatus(
    int64_t scan_ns, bool accepted, bool stationary, const std::string & reason,
    const MatchResult & fit, const std::optional<Eigen::Vector3d> & seed_radar = std::nullopt,
    const char * recovery_reason = nullptr)
  {
    if (!stationary || (!isVgicpRecoveryInput(reason) && !accepted)) {
      recovery_window_.reset();
    }
    const auto & review = fit.review;
    const Eigen::Vector3d * candidate = stationary && !accepted && review.winner ?
      &review.candidates[*review.winner].base_xy_yaw : nullptr;
    const auto streak = candidate_streak_.update(scan_ns, candidate);
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(9)
        << "{\"stamp_ns\":" << scan_ns
        << ",\"accepted\":" << (accepted ? "true" : "false")
        << ",\"stationary\":" << (stationary ? "true" : "false")
        << ",\"reason\":\"" << reason << "\""
        << ",\"overlap_8cm\":" << fit.overlap_8cm
        << ",\"inlier_ratio\":" << fit.inlier_ratio
        << ",\"geom_ratio\":" << fit.geometry_singular_ratio
        << ",\"seed_spread_m\":" << fit.seed_spread_m
        << ",\"solve_ms\":" << fit.solve_ms
        << ",\"base_xy_yaw\":[" << fit.base_xy_yaw.x() << "," << fit.base_xy_yaw.y()
        << "," << fit.base_xy_yaw.z() << "]";
    if (seed_radar) {
      oss << ",\"seed_radar_xy_yaw\":[" << seed_radar->x() << "," << seed_radar->y()
          << "," << seed_radar->z() << "]";
    }
    if (recovery_reason) {
      oss << ",\"stationary_recovery\":\"" << recovery_reason << "\"";
    }
    if (review_ambiguity_ && review.reason != "not_requested") {
      oss << ",\"candidate_review\":{\"reason\":\"" << review.reason << "\""
          << ",\"supporting_cells\":" << review.supporting_cells
          << ",\"margin_without_best_cell_m\":" << review.margin_without_best_cell_m
          << ",\"local_minimum_margin_m\":" << review.local_minimum_margin_m
          << ",\"streak\":" << streak
          << ",\"consistent\":" << (candidate_streak_.consistent() ? "true" : "false")
          << ",\"winner\":";
      if (review.winner) {
        oss << *review.winner;
      } else {
        oss << "null";
      }
      oss << ",\"candidates\":[";
      for (size_t i = 0; i < review.candidates.size(); ++i) {
        const auto & score = review.candidates[i];
        if (i > 0) {
          oss << ",";
        }
        oss << "{\"pose\":[" << score.base_xy_yaw.x() << "," << score.base_xy_yaw.y()
            << "," << score.base_xy_yaw.z() << "],\"overlap_8cm\":" << score.overlap_8cm
            << ",\"balanced_rms_m\":" << score.balanced_rms_m << "}";
      }
      oss << "]}";
    }
    oss << "}";
    std_msgs::msg::String msg;
    msg.data = oss.str();
    status_pub_->publish(msg);
  }

  std::unique_ptr<Vgicp4DofMatcher> matcher_;
  SE2 map_from_odin_map_{};
  bool use_odin_seed_{true};
  bool review_ambiguity_{true};
  VgicpCandidateStreak candidate_streak_;
  bool stationary_recovery_{true};
  double recovery_min_height_{0.70};
  VgicpRecoveryWindow recovery_window_;
  uint64_t recovery_epoch_{0};
  double tf_stamp_lead_{0.30};
  double max_pose_age_{0.50};
  double max_fix_timeout_{8.0};
  double max_fix_distance_{0.8};
  double max_fix_turn_{0.50};
  double cold_innovation_xy_{0.50};
  double cold_innovation_yaw_{0.45};
  double moving_fusion_alpha_{0.45};
  double moving_max_step_xy_{0.05};
  double moving_max_step_yaw_{0.05};
  double stationary_v_max_{0.015};
  double stationary_w_max_{0.030};
  int scan_stride_{5};
  std::string global_frame_{"map"};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  std::string lidar_frame_{"lidar"};

  rclcpp::CallbackGroup::SharedPtr io_group_;
  rclcpp::CallbackGroup::SharedPtr scan_group_;

  std::mutex odin_tf_mutex_;
  std::unique_ptr<tf2_ros::Buffer> odin_tf_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::mutex tf_emit_mutex_;
  int64_t last_tf_emit_ns_{0};

  std::mutex state_mutex_;
  std::deque<OdomSample> odom_history_;
  SE2 map_from_odom_{};
  double fix_distance_origin_{0.0};
  double fix_turn_origin_{0.0};
  uint64_t localization_epoch_{0};
  bool has_lock_{false};
  int64_t last_fix_ns_{0};
  std::optional<SE2> pending_initial_base_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr odin_tf_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr odin_tf_static_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace
}  // namespace odin_nav_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<odin_nav_sim::VgicpLocalizationNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
