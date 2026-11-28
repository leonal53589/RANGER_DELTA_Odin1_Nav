#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "angles/angles.h"
#include "settled_goal_checker.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "terminal_clearance.hpp"
#include "terminal_parking.hpp"

namespace odin_nav_sim
{

using nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController;

class DeltaPursuitController : public RegulatedPurePursuitController
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    RegulatedPurePursuitController::configure(parent, name, tf, costmap_ros);
    auto node = parent.lock();
    endpoint_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
      "~/tracking_endpoint", rclcpp::QoS(1).transient_local());
    accepted_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>(
      "~/accepted_plan", rclcpp::QoS(1).transient_local());
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    const auto tracking_parameter = name + ".tracking_controller";
    if (!node->has_parameter(tracking_parameter)) {
      node->declare_parameter(tracking_parameter, std::string{}, descriptor);
    }
    const auto tracking_plugin = node->get_parameter(tracking_parameter).as_string();
    if (!tracking_plugin.empty()) {
      tracking_controller_ = controller_loader_.createUniqueInstance(tracking_plugin);
      tracking_controller_->configure(parent, name + ".tracking", tf, costmap_ros);
      RCLCPP_INFO(logger_, "Using %s for path tracking", tracking_plugin.c_str());
    }
    const auto parameter = name + ".minimum_turn_radius";
    if (!node->has_parameter(parameter)) {
      node->declare_parameter(parameter, 0.5, descriptor);
    }
    minimum_turn_radius_ = node->get_parameter(parameter).as_double();
    if (!std::isfinite(minimum_turn_radius_) || minimum_turn_radius_ <= 0.0) {
      throw std::invalid_argument("minimum_turn_radius must be finite and positive");
    }
    const auto angular_parameter = name + ".max_tracking_angular_speed";
    if (!node->has_parameter(angular_parameter)) {
      node->declare_parameter(angular_parameter, 0.0, descriptor);
    }
    max_tracking_angular_speed_ = node->get_parameter(angular_parameter).as_double();
    if (!std::isfinite(max_tracking_angular_speed_) || max_tracking_angular_speed_ < 0.0) {
      throw std::invalid_argument("max_tracking_angular_speed must be finite and nonnegative");
    }
    const auto approach_parameter = name + ".max_approach_linear_speed";
    if (!node->has_parameter(approach_parameter)) {
      node->declare_parameter(approach_parameter, 0.0, descriptor);
    }
    max_approach_linear_speed_ = node->get_parameter(approach_parameter).as_double();
    if (!std::isfinite(max_approach_linear_speed_) || max_approach_linear_speed_ < 0.0) {
      throw std::invalid_argument("max_approach_linear_speed must be finite and nonnegative");
    }
    const auto topic_parameter = name + ".reposition_cloud_topic";
    if (!node->has_parameter(topic_parameter)) {
      node->declare_parameter(topic_parameter, std::string{}, descriptor);
    }
    const auto topic = node->get_parameter(topic_parameter).as_string();
    if (!topic.empty()) {
      cloud_sub_ = node->create_subscription<Cloud>(
        topic, rclcpp::SensorDataQoS().keep_last(1),
        [this](Cloud::ConstSharedPtr cloud) {std::atomic_store(&cloud_, std::move(cloud));});
    }
    const auto settle_hold_param = name + ".terminal_settle_hold_time";
    if (!node->has_parameter(settle_hold_param)) {
      node->declare_parameter(settle_hold_param, 0.30, descriptor);
    }
    terminal_settle_hold_time_ = node->get_parameter(settle_hold_param).as_double();
    if (!std::isfinite(terminal_settle_hold_time_) || terminal_settle_hold_time_ <= 0.0) {
      throw std::invalid_argument("terminal_settle_hold_time must be finite and positive");
    }
    const auto settle_timeout_param = name + ".terminal_settle_timeout";
    if (!node->has_parameter(settle_timeout_param)) {
      node->declare_parameter(settle_timeout_param, 3.0, descriptor);
    }
    terminal_settle_timeout_ = node->get_parameter(settle_timeout_param).as_double();
    if (!std::isfinite(terminal_settle_timeout_) || terminal_settle_timeout_ <= 0.0) {
      throw std::invalid_argument("terminal_settle_timeout must be finite and positive");
    }
    parking_ = TerminalParking(terminal_settle_hold_time_, terminal_settle_timeout_);
  }

  void cleanup() override
  {
    if (tracking_controller_) {
      tracking_controller_->cleanup();
      tracking_controller_.reset();
    }
    accepted_plan_pub_.reset();
    endpoint_pub_.reset();
    cloud_sub_.reset();
    std::atomic_store(&cloud_, Cloud::ConstSharedPtr{});
    recent_clouds_.clear();
    recent_footprints_.clear();
    RegulatedPurePursuitController::cleanup();
  }

  void activate() override
  {
    RegulatedPurePursuitController::activate();
    endpoint_pub_->on_activate();
    accepted_plan_pub_->on_activate();
    if (tracking_controller_) {
      tracking_controller_->activate();
    }
  }

  void deactivate() override
  {
    if (tracking_controller_) {
      tracking_controller_->deactivate();
    }
    accepted_plan_pub_->on_deactivate();
    endpoint_pub_->on_deactivate();
    RegulatedPurePursuitController::deactivate();
  }

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override
  {
    RegulatedPurePursuitController::setSpeedLimit(speed_limit, percentage);
    if (tracking_controller_) {
      tracking_controller_->setSpeedLimit(speed_limit, percentage);
    }
  }

  void setPlan(const nav_msgs::msg::Path & path) override
  {
    std::lock_guard<std::mutex> controller_lock(mutex_);
    if (path.poses.empty()) {
      throw nav2_core::PlannerException("DeltaPursuit received an empty path");
    }
    auto endpoint = path.poses.back();
    endpoint.header.frame_id = path.header.frame_id;
    const auto & previous = endpoint_.pose;
    const auto & next = endpoint.pose;
    if (endpoint_.header.frame_id != endpoint.header.frame_id ||
      std::hypot(previous.position.x - next.position.x, previous.position.y - next.position.y) >
      1e-6 || std::abs(angles::shortest_angular_distance(
        tf2::getYaw(previous.orientation), tf2::getYaw(next.orientation))) > 1e-6 ||
      parking_finished_ || Clock::now() - last_compute_ > std::chrono::seconds(1))
    {
      parking_ = TerminalParking(terminal_settle_hold_time_, terminal_settle_timeout_);
      parking_finished_ = false;
      parking_failed_ = false;
      parking_failure_reason_.clear();
      reposition_mode_ = RepositionMode::Select;
      reposition_switches_ = 0;
      reposition_clearance_misses_ = 0;
      reposition_path_started_.reset();
      reposition_path_min_dist_ = std::numeric_limits<double>::infinity();
      min_approach_dist_ = std::numeric_limits<double>::infinity();
      diag_dir_x_ = 0.0;
      diag_dir_y_ = 0.0;
      last_parking_command_ = TerminalParking::Command::FollowPath;
      last_pose_stamp_ = 0;
    }
    endpoint_ = endpoint;
    RegulatedPurePursuitController::setPlan(path);
    if (tracking_controller_) {
      tracking_controller_->setPlan(path);
    }
    endpoint_pub_->publish(endpoint_);
    accepted_plan_pub_->publish(path);
    phase_ = Phase::Tracking;
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & speed,
    nav2_core::GoalChecker * goal_checker) override
  {
    std::lock_guard<std::mutex> controller_lock(mutex_);
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*costmap_->getMutex());
    last_compute_ = Clock::now();
    auto * settled_checker = dynamic_cast<SettledGoalChecker *>(goal_checker);
    if (settled_checker) {
      settled_checker->setReached(false);
    }
    if (parking_failed_) {
      throw nav2_core::PlannerException(parking_failure_reason_);
    }
    geometry_msgs::msg::Pose pose_tolerance;
    geometry_msgs::msg::Twist velocity_tolerance;
    double goal_yaw_tolerance = 0.10;
    if (goal_checker->getTolerances(pose_tolerance, velocity_tolerance)) {
      goal_dist_tol_ = pose_tolerance.position.x;
      goal_yaw_tolerance = std::abs(tf2::getYaw(pose_tolerance.orientation));
    }

    const auto & fp_pos = pose.pose.position;
    const double fp_yaw = tf2::getYaw(pose.pose.orientation);
    if (recent_footprints_.empty() ||
      std::hypot(fp_pos.x - recent_footprints_.back().x, fp_pos.y - recent_footprints_.back().y) >= 0.02)
    {
      recent_footprints_.push_back({last_compute_, fp_pos.x, fp_pos.y});
    }
    while (!recent_footprints_.empty() &&
      last_compute_ - recent_footprints_.front().stamp > std::chrono::seconds(45))
    {
      recent_footprints_.pop_front();
    }
    if (const auto live_cloud = std::atomic_load(&cloud_)) {
      const auto cloud_stamp = rclcpp::Time(live_cloud->header.stamp);
      const auto cloud_age = (clock_->now() - cloud_stamp).seconds();
      const bool same_stamp = !recent_clouds_.empty() &&
        rclcpp::Time(recent_clouds_.back().cloud->header.stamp) == cloud_stamp;
      const bool should_capture = recent_clouds_.empty() ||
        (!same_stamp &&
        (std::hypot(fp_pos.x - recent_clouds_.back().x, fp_pos.y - recent_clouds_.back().y) >= 0.10 ||
        std::abs(angles::shortest_angular_distance(recent_clouds_.back().yaw, fp_yaw)) >= 0.30 ||
        (last_compute_ - recent_clouds_.back().stamp) >= std::chrono::milliseconds(3000)));
      if (cloud_stamp.nanoseconds() != 0 && cloud_age >= 0.0 && cloud_age <= 0.50 &&
        !live_cloud->header.frame_id.empty() && should_capture)
      {
        tf2::Transform cloud_to_frame;
        bool has_tf = false;
        try {
          const auto transform = tf_->lookupTransform(
            pose.header.frame_id, live_cloud->header.frame_id, cloud_stamp,
            rclcpp::Duration::from_seconds(0.0));
          tf2::fromMsg(transform.transform, cloud_to_frame);
          has_tf = true;
        } catch (const tf2::TransformException &) {
          has_tf = false;
        }
        // Store the odom transform at capture time so 45s approach scans outlive the 10s TF buffer.
        if (has_tf) {
          recent_clouds_.push_back({
            last_compute_, fp_pos.x, fp_pos.y, fp_yaw, live_cloud,
            pose.header.frame_id, cloud_to_frame, true});
        }
      }
      while (recent_clouds_.size() > 24 || (!recent_clouds_.empty() &&
        last_compute_ - recent_clouds_.front().stamp > std::chrono::seconds(45)))
      {
        recent_clouds_.pop_front();
      }
    }

    const auto path = transformGlobalPlan(pose);
    double lookahead = getLookAheadDistance(speed);
    if (allow_reversing_) {
      lookahead = std::min(lookahead, findVelocitySignChange(path));
    }
    const auto carrot = getLookAheadPoint(lookahead, path);
    carrot_pub_->publish(createCarrotMsg(carrot));
    const auto & point = carrot.pose.position;
    const double distance = std::hypot(point.x, point.y);
    const double path_heading = std::atan2(point.y, point.x);
    const double curvature_limit = 1.0 / minimum_turn_radius_;
    const double feasible_heading = std::asin(std::min(1.0, distance * curvature_limit / 2.0));
    const double align_enter = std::min(rotate_to_heading_min_angle_,
        std::max(0.18, feasible_heading));
    const double align_exit = std::min(0.15, 0.55 * align_enter);
    using Command = TerminalParking::Command;
    auto parking_command = Command::FollowPath;
    const auto & end = path.poses.back().pose;
    double goal_yaw = tf2::getYaw(end.orientation);
    double goal_distance = std::hypot(end.position.x, end.position.y);
    std::optional<PlanarVelocity> reposition_vel;
    if (settled_checker) {
      const auto parking = updateParking(pose, speed, goal_yaw_tolerance);
      parking_command = parking.command;
      goal_yaw = parking.heading_error;
      goal_distance = std::hypot(parking.longitudinal, parking.lateral);
      if (!parking_.isRepositioning()) {
        reposition_mode_ = RepositionMode::Select;
        reposition_switches_ = 0;
        reposition_clearance_misses_ = 0;
        reposition_path_started_.reset();
        reposition_path_min_dist_ = std::numeric_limits<double>::infinity();
      }
      if (parking_command != last_parking_command_) {
        if (parking_command == Command::Reposition) {
          RCLCPP_INFO(
            logger_,
            "Terminal parking: repositioning toward the path endpoint "
            "(dx=%.4f, dy=%.4f, dyaw=%.2fdeg, inner=%.4f)",
            parking.longitudinal, parking.lateral,
            angles::to_degrees(parking.heading_error),
            TerminalParking::repositionToleranceFactor(
              parking.heading_error, goal_yaw_tolerance) * goal_dist_tol_);
        } else if (parking_command == Command::Reached) {
          RCLCPP_INFO(
            logger_,
            "Terminal parking: position and heading verified at rest "
            "(err=%.4fm, dx=%.4f, dy=%.4f, dyaw=%.2fdeg)",
            goal_distance, parking.longitudinal, parking.lateral,
            angles::to_degrees(parking.heading_error));
        }
        last_parking_command_ = parking_command;
      }
      if (parking_command == Command::Failed) {
        parking_finished_ = true;
        parking_failed_ = true;
        parking_failure_reason_ = "Terminal parking could not settle within its limits";
        throw nav2_core::PlannerException(parking_failure_reason_);
      }
      if (parking_command == Command::Reposition) {
        reposition_vel = repositionVelocity(pose, speed, parking, goal_yaw_tolerance);
      }
      if (!reposition_vel &&
        (parking_command == Command::FollowPath || parking_command == Command::Reposition))
      {
        if (parking_command == Command::FollowPath && goal_distance <= goal_dist_tol_) {
          // Keep gliding straight toward the center without triggering an in-place carrot spin.
          phase_ = Phase::Tracking;
        } else {
          // At the terminal carrot a small bearing error can still intersect the
          // arrival disk on a forward arc. Avoid spinning just to hit its center.
          const bool terminal_carrot = parking_command == Command::FollowPath &&
            goal_distance < 0.30 && goal_distance > goal_dist_tol_ &&
            std::hypot(point.x - parking.longitudinal, point.y - parking.lateral) < 0.005;
          if (terminal_carrot) {
            const double disk_angle = std::asin(kRepositionToleranceFactor * goal_dist_tol_ / goal_distance);
            updatePathPhase(path_heading, align_enter + disk_angle, std::min(align_exit, 0.035));
          } else {
            updatePathPhase(path_heading, align_enter, align_exit);
          }
        }
      } else {
        phase_ = Phase::Tracking;
      }
    } else {
      updatePhase(
        goal_distance, goal_yaw, goal_yaw_tolerance,
        path_heading, align_enter, align_exit);
    }

    double linear = desired_linear_vel_;
    double lateral = 0.0;
    double angular = 0.0;

    if (reposition_vel) {
      linear = reposition_vel->vx;
      lateral = reposition_vel->vy;
    } else if (parking_command == Command::Hold || parking_command == Command::Reached) {
      linear = 0.0;
    } else if (parking_command == Command::AlignGoal || phase_ == Phase::AlignGoal) {
      rotateToHeading(linear, angular, goal_yaw, speed);
      if (settled_checker && angular != 0.0) {
        // Keep rotational drive above static friction while capping spin-in-place near the goal.
        const double desired_mag = std::clamp(1.8 * std::abs(goal_yaw), 0.08, 0.35);
        const double limited = std::min(std::abs(angular), desired_mag);
        angular = std::copysign(limited, angular);
      }
    } else if (phase_ == Phase::AlignPath) {
      rotateToHeading(linear, angular, path_heading, speed);
    } else if (tracking_controller_ && parking_command == Command::FollowPath) {
      bool spin_fallback = false;
      try {
        const auto tracking = tracking_controller_->computeVelocityCommands(
          pose, speed, goal_checker);
        if (!std::isfinite(tracking.twist.linear.x) || !std::isfinite(tracking.twist.angular.z)) {
          throw nav2_core::PlannerException("Tracking controller returned a non-finite velocity");
        }
        double speed_limit = std::abs(desired_linear_vel_);
        if (max_approach_linear_speed_ > 0.0 && approachVelocityScalingFactor(path) < 1.0) {
          speed_limit = std::min(speed_limit, max_approach_linear_speed_);
        }
        if (settled_checker) {
          // Continuous speed cap down to the edge of the arrival tolerance (m/s).
          const double remaining = std::max(0.0, goal_distance - goal_dist_tol_);
          speed_limit = std::min(speed_limit, 0.035 + 0.55 * remaining);
        }
        // Enforce chassis limits after the optimizer smooths the control sequence.
        linear = std::max(0.0, tracking.twist.linear.x);
        const double max_yaw_rate = linear * curvature_limit;
        angular = std::clamp(tracking.twist.angular.z, -max_yaw_rate, max_yaw_rate);
        if (linear > speed_limit) {
          angular *= speed_limit / linear;
          linear = speed_limit;
        }
        // Below the drive deadband the chassis would interpret yaw as a spin request.
        if (linear < 0.001) {
          if (use_rotate_to_heading_ && std::abs(path_heading) > 0.15) {
            spin_fallback = true;
          } else {
            linear = 0.0;
            angular = 0.0;
          }
        }
      } catch (const nav2_core::PlannerException &) {
        if (!use_rotate_to_heading_ || std::abs(path_heading) <= 0.12) {
          throw;
        }
        spin_fallback = true;
      }
      if (spin_fallback) {
        rotateToHeading(linear, angular, path_heading, speed);
      }
    } else {
      const double curvature = distance > 0.0 ? 2.0 * point.y / (distance * distance) : 0.0;
      double direction = allow_reversing_ && point.x < 0.0 ? -1.0 : 1.0;
      if (max_approach_linear_speed_ > 0.0 && approachVelocityScalingFactor(path) < 1.0) {
        linear = std::min(linear, max_approach_linear_speed_);
      }
      applyConstraints(
        curvature, speed, costAtPose(pose.pose.position.x, pose.pose.position.y), path,
        linear, direction);
      if (parking_command == Command::Reposition) {
        linear = std::clamp(linear, -kAxialSpeed, kAxialSpeed);
      }
      angular = linear * std::clamp(curvature, -curvature_limit, curvature_limit);
    }

    if (linear != 0.0 && max_tracking_angular_speed_ > 0.0 &&
      std::abs(angular) > max_tracking_angular_speed_)
    {
      // Preserve the turning radius when limiting speed, including in the collision check.
      linear *= max_tracking_angular_speed_ / std::abs(angular);
      angular = std::copysign(max_tracking_angular_speed_, angular);
    }
    if (use_collision_detection_) {
      checkTrajectory(pose, distance, linear, lateral, angular);
    }
    if (parking_command == Command::Reached) {
      parking_finished_ = true;
      settled_checker->setReached(true);
    }
    geometry_msgs::msg::TwistStamped command;
    command.header = pose.header;
    command.twist.linear.x = linear;
    command.twist.linear.y = lateral;
    command.twist.angular.z = angular;
    return command;
  }

private:
  using Clock = std::chrono::steady_clock;
  using Cloud = sensor_msgs::msg::PointCloud2;
  enum class Phase {Tracking, AlignPath, AlignGoal};
  enum class RepositionMode {Select, Path, Forward, Reverse, Left, Right, Diagonal};
  struct PlanarVelocity { double vx; double vy; };

  struct ParkingUpdate
  {
    TerminalParking::Command command;
    double heading_error;
    double longitudinal;
    double lateral;
  };

  ParkingUpdate updateParking(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & speed,
    double goal_yaw_tolerance)
  {
    auto endpoint = endpoint_;
    endpoint.header.stamp = pose.header.stamp;
    geometry_msgs::msg::PoseStamped goal;
    if (!transformPose(pose.header.frame_id, endpoint, goal)) {
      parking_finished_ = true;
      throw nav2_core::PlannerException("Cannot transform parking endpoint at the robot pose time");
    }
    const double dx = goal.pose.position.x - pose.pose.position.x;
    const double dy = goal.pose.position.y - pose.pose.position.y;
    const double body_yaw = tf2::getYaw(pose.pose.orientation);
    const double longitudinal = dx * std::cos(body_yaw) + dy * std::sin(body_yaw);
    const double lateral = -dx * std::sin(body_yaw) + dy * std::cos(body_yaw);
    const double yaw = angles::shortest_angular_distance(
      body_yaw, tf2::getYaw(goal.pose.orientation));
    const auto stamp = rclcpp::Time(pose.header.stamp).nanoseconds();
    const bool fresh = stamp > last_pose_stamp_;
    last_pose_stamp_ = std::max(last_pose_stamp_, stamp);
    const bool stopped = std::hypot(speed.linear.x, speed.linear.y) <= 0.01 &&
      std::abs(speed.angular.z) <= 0.03;
    const auto time = std::chrono::duration<double>(last_compute_.time_since_epoch()).count();
    const double dist = std::hypot(dx, dy);
    double effective_xy_tol = goal_dist_tol_;
    if (parking_.isApproaching() && dist <= goal_dist_tol_) {
      min_approach_dist_ = std::min(min_approach_dist_, dist);
      // Let an inward approach glide closer to the center so spin-center offset
      // during final heading alignment rarely pushes the chassis back out of tolerance.
      const bool gliding_inward = dist > 0.55 * goal_dist_tol_ && longitudinal > 0.005 &&
        std::abs(std::atan2(lateral, longitudinal)) < 0.40 &&
        speed.linear.x > 0.015 && dist <= min_approach_dist_ + 0.001;
      if (gliding_inward) {
        effective_xy_tol = 0.55 * goal_dist_tol_;
      }
    }
    return {parking_.update({time, dist, yaw, effective_xy_tol, goal_yaw_tolerance,
        stopped, fresh}), yaw, longitudinal, lateral};
  }

  static const char * clearanceReasonName(TerminalClearance::RejectReason reason)
  {
    using R = TerminalClearance::RejectReason;
    switch (reason) {
      case R::Cleared: return "cleared";
      case R::NoCloud: return "no_cloud";
      case R::StaleCloud: return "stale_cloud";
      case R::TfFailure: return "tf_failure";
      case R::OutOfBounds: return "out_of_bounds";
      case R::CostmapLethal: return "costmap_lethal";
      case R::CloudLethal: return "cloud_lethal";
      case R::Unobserved: return "unobserved";
      case R::NoNewCells: return "no_new_cells";
    }
    return "unknown";
  }

  static std::string formatClearanceReport(
    const char * label, double sweep, double body_angle_deg,
    const TerminalClearance::Report & r)
  {
    std::ostringstream oss;
    oss << std::fixed;
    oss.precision(3);
    oss << label << "(sweep=" << sweep << ",ang=";
    oss.precision(1);
    oss << body_angle_deg << "deg):" << clearanceReasonName(r.reason);
    oss.precision(3);
    using R = TerminalClearance::RejectReason;
    if (r.reason == R::CostmapLethal || r.reason == R::CloudLethal || r.reason == R::Unobserved) {
      oss << "@(" << r.block_x << "," << r.block_y << ")dist=" << r.block_dist
          << ",cost=" << r.costmap_cost << ",z=" << r.cloud_z << ",hits=" << r.cloud_hits;
    } else if (r.reason == R::StaleCloud) {
      oss << "(age=" << r.cloud_age << "s)";
    }
    oss << "[swept=" << r.swept_cells << ",free=" << r.free_cells
        << ",unk=" << r.unknown_cells << ",priors=" << r.prior_scans_ok
        << "/" << r.prior_scans_fed << ",pts=" << r.live_points << "]";
    return oss.str();
  }

  std::optional<PlanarVelocity> repositionVelocity(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & speed,
    const ParkingUpdate & parking, double yaw_tolerance)
  {
    const double dx = parking.longitudinal;
    const double dy = parking.lateral;
    const double dist = std::hypot(dx, dy);
    if (reposition_mode_ == RepositionMode::Path) {
      if (!reposition_path_started_) {
        reposition_path_started_ = last_compute_;
        reposition_path_min_dist_ = dist;
      } else if (dist <= reposition_path_min_dist_ - 0.005) {
        reposition_path_min_dist_ = dist;
        reposition_path_started_ = last_compute_;
      } else if (last_compute_ - *reposition_path_started_ >= std::chrono::milliseconds(2500)) {
        parking_finished_ = true;
        parking_failed_ = true;
        parking_failure_reason_ = "Terminal parking path repositioning stalled";
        RCLCPP_WARN(
          logger_,
          "Terminal parking: path repositioning stalled for 2.5s "
          "(dx=%.3f, dy=%.3f, best_dist=%.3f)",
          dx, dy, reposition_path_min_dist_);
        throw nav2_core::PlannerException(parking_failure_reason_);
      }
      return std::nullopt;
    }
    const double inner_tol =
      TerminalParking::repositionToleranceFactor(parking.heading_error, yaw_tolerance) *
      goal_dist_tol_;
    const double yaw_limit = std::max(yaw_tolerance, 0.50);
    std::string reject_log;
    auto appendReject = [&reject_log](const std::string & item) {
        if (!reject_log.empty()) {
          reject_log += "; ";
        }
        reject_log += item;
      };

    auto trySelectAxis = [&](bool lateral) -> bool {
        const double err = lateral ? dy : dx;
        const double other = lateral ? dx : dy;
        const auto direct = axialSweepDistance(err, other, inner_tol);
        std::optional<double> sweep = direct;
        if (!sweep && std::abs(err) >= 0.5 * inner_tol &&
          dist <= kMaxAxialTravel &&
          std::abs(err) + kAxialStoppingMargin <= kMaxAxialTravel + kAxialStoppingMargin)
        {
          sweep = std::copysign(std::abs(err) + kAxialStoppingMargin, err);
        }
        const double body_angle = lateral ? M_PI_2 : 0.0;
        const char * label = lateral ?
          (err >= 0.0 ? "left crab" : "right crab") :
          (err >= 0.0 ? "forward" : "reverse");
        if (!sweep) {
          std::ostringstream oss;
          oss << std::fixed;
          oss.precision(3);
          oss << label << ":no_sweep(err=" << err << ",other=" << other
              << ",inner=" << inner_tol << ")";
          appendReject(oss.str());
          return false;
        }
        TerminalClearance::Report report;
        if (!hasObservedClearance(pose, *sweep, body_angle, &report)) {
          appendReject(
            formatClearanceReport(label, *sweep, angles::to_degrees(body_angle), report));
          return false;
        }
        reposition_mode_ = lateral ?
          (err >= 0.0 ? RepositionMode::Left : RepositionMode::Right) :
          (err >= 0.0 ? RepositionMode::Forward : RepositionMode::Reverse);
        axial_start_ = pose.pose.position;
        RCLCPP_INFO(
          logger_, "Terminal parking: straight %s correction (dx=%.3f, dy=%.3f)",
          label, dx, dy);
        return true;
      };

    auto trySelectDiagonal = [&]() -> bool {
        if (dist <= inner_tol || dist > kMaxAxialTravel) {
          return false;
        }
        const double ux = dx / dist;
        const double uy = dy / dist;
        if (std::abs(ux) < 0.15 || std::abs(uy) < 0.15) {
          appendReject("diag:axis_aligned");
          return false;
        }
        // Ranger Delta CRAB handles physical steering in [-90 deg, +30 deg] (wrapped via reverse drive).
        const double sx = dx >= 0.0 ? dx : -dx;
        const double sy = dx >= 0.0 ? dy : -dy;
        const double steer = std::atan2(sy, sx);
        if (steer < -M_PI_2 || steer > 0.48) {
          std::ostringstream oss;
          oss << std::fixed;
          oss.precision(1);
          oss << "diag:steer_out(" << angles::to_degrees(steer) << "deg)";
          appendReject(oss.str());
          return false;
        }
        const double body_angle = std::atan2(dy, dx);
        const double sweep_diag = std::max(dist - inner_tol + kAxialStoppingMargin, 0.035);
        TerminalClearance::Report report;
        if (!hasObservedClearance(pose, sweep_diag, body_angle, &report)) {
          appendReject(
            formatClearanceReport("diag", sweep_diag, angles::to_degrees(body_angle), report));
          return false;
        }
        reposition_mode_ = RepositionMode::Diagonal;
        diag_dir_x_ = ux;
        diag_dir_y_ = uy;
        axial_start_ = pose.pose.position;
        RCLCPP_INFO(
          logger_, "Terminal parking: diagonal crab correction (dx=%.3f, dy=%.3f)", dx, dy);
        return true;
      };

    if (reposition_mode_ == RepositionMode::Select) {
      bool selected = false;
      const bool geometry_within_reach = dist <= kMaxAxialTravel &&
        std::abs(parking.heading_error) < yaw_limit;
      if (geometry_within_reach && use_collision_detection_) {
        if (std::abs(dy) < inner_tol && axialSweepDistance(dx, dy, inner_tol)) {
          selected = trySelectAxis(false);
        } else if (std::abs(dx) < inner_tol && axialSweepDistance(dy, dx, inner_tol)) {
          selected = trySelectAxis(true);
        } else {
          selected = trySelectDiagonal();
          if (!selected) {
            if (std::abs(dy) >= std::abs(dx)) {
              selected = trySelectAxis(true) || trySelectAxis(false);
            } else {
              selected = trySelectAxis(false) || trySelectAxis(true);
            }
          }
        }
      }
      if (!selected) {
        reposition_mode_ = RepositionMode::Path;
        reposition_path_started_ = last_compute_;
        reposition_path_min_dist_ = dist;
        RCLCPP_INFO(
          logger_,
          "Terminal parking: following path for repositioning "
          "(dx=%.3f, dy=%.3f, dyaw=%.2fdeg, inner=%.3f; %s)%s%s",
          dx, dy, angles::to_degrees(parking.heading_error), inner_tol,
          geometry_within_reach ? "no verified straight clearance" : "outside straight-move limits",
          reject_log.empty() ? "" : " | ", reject_log.c_str());
        return std::nullopt;
      }
    }

    double remaining = 0.0;
    double stop_thresh = 0.5 * inner_tol;
    double body_angle = 0.0;
    double dir_x = 0.0;
    double dir_y = 0.0;
    double max_speed = std::min(kAxialSpeed, std::abs(desired_linear_vel_));

    if (reposition_mode_ == RepositionMode::Diagonal) {
      remaining = dx * diag_dir_x_ + dy * diag_dir_y_;
      const double perp = std::abs(-dx * diag_dir_y_ + dy * diag_dir_x_);
      stop_thresh = (perp < inner_tol) ?
        std::sqrt(std::max(0.0, inner_tol * inner_tol - perp * perp)) : 0.5 * inner_tol;
      body_angle = std::atan2(diag_dir_y_, diag_dir_x_);
      dir_x = diag_dir_x_;
      dir_y = diag_dir_y_;
      max_speed = kCrabSpeed;
    } else {
      const bool lateral =
        reposition_mode_ == RepositionMode::Left || reposition_mode_ == RepositionMode::Right;
      const double err = lateral ? dy : dx;
      const double other = lateral ? dx : dy;
      const double sign =
        (reposition_mode_ == RepositionMode::Forward || reposition_mode_ == RepositionMode::Left) ?
        1.0 : -1.0;
      stop_thresh = (std::abs(other) < inner_tol) ?
        std::sqrt(std::max(0.0, inner_tol * inner_tol - other * other)) : 0.5 * inner_tol;
      remaining = sign * err;
      body_angle = lateral ? M_PI_2 : 0.0;
      dir_x = lateral ? 0.0 : sign;
      dir_y = lateral ? sign : 0.0;
      max_speed = lateral ? kCrabSpeed : max_speed;
    }

    if (remaining <= stop_thresh) {
      const bool stopped = std::hypot(speed.linear.x, speed.linear.y) <= 0.01 &&
        std::abs(speed.angular.z) <= 0.03;
      if (stopped) {
        if (++reposition_switches_ > 3) {
          reposition_mode_ = RepositionMode::Path;
          reposition_path_started_ = last_compute_;
          reposition_path_min_dist_ = dist;
          RCLCPP_INFO(
            logger_,
            "Terminal parking: max reposition switches (%d) reached, switching to path "
            "(dx=%.3f, dy=%.3f)",
            reposition_switches_, dx, dy);
          return std::nullopt;
        }
        reposition_mode_ = RepositionMode::Select;
        reposition_clearance_misses_ = 0;
      }
      return PlanarVelocity{0.0, 0.0};
    }

    const double traveled = std::hypot(
      pose.pose.position.x - axial_start_.x, pose.pose.position.y - axial_start_.y);
    const double dist_left = remaining - stop_thresh;
    const double sweep_mag = std::max(dist_left + kAxialStoppingMargin, 0.035);
    const double sweep_check =
      (reposition_mode_ == RepositionMode::Forward || reposition_mode_ == RepositionMode::Left ||
      reposition_mode_ == RepositionMode::Diagonal) ? sweep_mag : -sweep_mag;
    if (std::abs(parking.heading_error) >= yaw_limit ||
      traveled + remaining > kMaxAxialTravel + 0.05 || !use_collision_detection_)
    {
      RCLCPP_WARN(
        logger_,
        "Terminal parking aborting straight move: yaw=%.2fdeg (limit=%.2fdeg), "
        "traveled=%.3f, remaining=%.3f, dx=%.3f, dy=%.3f",
        angles::to_degrees(parking.heading_error), angles::to_degrees(yaw_limit),
        traveled, remaining, dx, dy);
      parking_finished_ = true;
      parking_failed_ = true;
      parking_failure_reason_ = "Terminal parking lost straight-move clearance";
      throw nav2_core::PlannerException(parking_failure_reason_);
    }
    TerminalClearance::Report active_report;
    if (!hasObservedClearance(pose, sweep_check, body_angle, &active_report)) {
      ++reposition_clearance_misses_;
      RCLCPP_WARN(
        logger_,
        "Terminal parking straight clearance miss %d/3 (dx=%.3f, dy=%.3f, rem=%.3f, stop=%.3f): %s",
        reposition_clearance_misses_, dx, dy, remaining, stop_thresh,
        formatClearanceReport("active", sweep_check, angles::to_degrees(body_angle), active_report)
          .c_str());
      if (reposition_clearance_misses_ < 3) {
        return PlanarVelocity{0.0, 0.0};
      }
      parking_finished_ = true;
      parking_failed_ = true;
      parking_failure_reason_ = "Terminal parking lost straight-move clearance";
      throw nav2_core::PlannerException(parking_failure_reason_);
    }
    reposition_clearance_misses_ = 0;
    const double speed_mag = std::clamp(1.6 * dist_left, 0.015, max_speed);
    return PlanarVelocity{dir_x * speed_mag, dir_y * speed_mag};
  }

  bool hasObservedClearance(
    const geometry_msgs::msg::PoseStamped & pose, double sweep, double body_angle_offset = 0.0,
    TerminalClearance::Report * report = nullptr) const
  {
    if (report) {
      *report = TerminalClearance::Report{};
    }
    const auto cloud = std::atomic_load(&cloud_);
    if (!cloud || cloud->data.empty() || cloud->header.frame_id.empty() || cloud->is_bigendian ||
      cloud->height == 0 || cloud->width == 0 || cloud->point_step == 0 ||
      static_cast<size_t>(cloud->width) * cloud->point_step != cloud->row_step ||
      cloud->data.size() != static_cast<size_t>(cloud->row_step) * cloud->height)
    {
      if (report) {
        report->reason = TerminalClearance::RejectReason::NoCloud;
      }
      return false;
    }
    const auto stamp = rclcpp::Time(cloud->header.stamp);
    const auto age = (clock_->now() - stamp).seconds();
    if (stamp.nanoseconds() == 0 || age < 0.0 || age > 0.50) {
      if (report) {
        report->reason = TerminalClearance::RejectReason::StaleCloud;
        report->cloud_age = age;
      }
      return false;
    }
    TerminalClearance clearance(*costmap_);
    const double footprint_r = costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();
    for (const auto & fp : recent_footprints_) {
      clearance.markRecentlyOccupied(fp.x, fp.y, footprint_r);
    }

    auto feedCloud = [&](
      const Cloud::ConstSharedPtr & scan, bool mark_obstacle,
      const tf2::Transform * cached_transform = nullptr) -> bool {
        if (!scan || scan->data.empty() || scan->header.frame_id.empty() || scan->is_bigendian ||
          scan->height == 0 || scan->width == 0 || scan->point_step == 0 ||
          static_cast<size_t>(scan->width) * scan->point_step != scan->row_step ||
          scan->data.size() != static_cast<size_t>(scan->row_step) * scan->height)
        {
          return false;
        }
        const auto field_offset = [&scan](const char * name) -> std::optional<size_t> {
            const auto field = std::find_if(
              scan->fields.begin(), scan->fields.end(),
              [name](const auto & f) {return f.name == name;});
            if (field == scan->fields.end() ||
              field->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
              field->count != 1 || field->offset + sizeof(float) > scan->point_step)
            {
              return std::nullopt;
            }
            return field->offset;
          };
        const auto x_offset = field_offset("x"), y_offset = field_offset("y");
        const auto z_offset = field_offset("z");
        if (!x_offset || !y_offset || !z_offset) {
          return false;
        }
        tf2::Transform cloud_to_odom;
        if (cached_transform) {
          cloud_to_odom = *cached_transform;
        } else {
          const auto scan_stamp = rclcpp::Time(scan->header.stamp);
          try {
            const auto transform = tf_->lookupTransform(
              pose.header.frame_id, scan->header.frame_id, scan_stamp,
              rclcpp::Duration::from_seconds(0.0));
            tf2::fromMsg(transform.transform, cloud_to_odom);
          } catch (const tf2::TransformException &) {
            return false;
          }
        }
        const auto & origin = cloud_to_odom.getOrigin();
        const auto * data = scan->data.data();
        for (size_t i = 0; i < scan->data.size(); i += scan->point_step) {
          // ODIN1 packs points at a 19-byte stride, so XYZ need not be aligned.
          float x, y, z;
          std::memcpy(&x, data + i + *x_offset, sizeof(float));
          std::memcpy(&y, data + i + *y_offset, sizeof(float));
          std::memcpy(&z, data + i + *z_offset, sizeof(float));
          if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
            const tf2::Vector3 hit = cloud_to_odom * tf2::Vector3(x, y, z);
            if (mark_obstacle) {
              clearance.observe(origin, hit);
            } else {
              clearance.observeFreeOnly(origin, hit);
            }
          }
        }
        return true;
      };

    unsigned int prior_fed = 0;
    unsigned int prior_ok = 0;
    const std::size_t live_points = static_cast<std::size_t>(cloud->width) * cloud->height;

    // Incorporate up to six prior approach/spin snapshots strictly as free-space evidence
    // so side-rear blind zones seen during pull-in and turns remain verified for CRAB shifts,
    // while live obstacle hits always come from the latest scan and rolling costmap.
    if (!recent_clouds_.empty()) {
      const size_t n = recent_clouds_.size();
      const size_t sample_count = std::min<size_t>(n, 6);
      const Cloud * last_fed = nullptr;
      for (size_t s = 0; s < sample_count; ++s) {
        const size_t idx = (s * n) / sample_count;
        const auto & sample = recent_clouds_[idx];
        const Cloud::ConstSharedPtr & prior = sample.cloud;
        if (prior.get() != cloud.get() && prior.get() != last_fed) {
          ++prior_fed;
          const tf2::Transform * cached =
            (sample.has_transform && sample.frame_id == pose.header.frame_id) ?
            &sample.cloud_to_frame : nullptr;
          if (feedCloud(prior, false, cached)) {
            ++prior_ok;
          }
          last_fed = prior.get();
        }
      }
    }
    if (!feedCloud(cloud, true)) {
      if (report) {
        report->reason = TerminalClearance::RejectReason::TfFailure;
        report->cloud_age = age;
        report->prior_scans_fed = prior_fed;
        report->prior_scans_ok = prior_ok;
        report->live_points = live_points;
      }
      return false;
    }

    const double sweep_yaw = tf2::getYaw(pose.pose.orientation) + body_angle_offset;
    const bool ok = clearance.permits(
      *costmap_, pose.pose.position.x, pose.pose.position.y, sweep_yaw, sweep, footprint_r, report);
    if (report) {
      report->cloud_age = age;
      report->prior_scans_fed = prior_fed;
      report->prior_scans_ok = prior_ok;
      report->live_points = live_points;
    }
    return ok;
  }

  void updatePhase(
    double goal_distance, double goal_yaw, double goal_yaw_tolerance,
    double path_heading, double align_enter, double align_exit)
  {
    if (!use_rotate_to_heading_) {
      phase_ = Phase::Tracking;
      return;
    }
    if (goal_distance < goal_dist_tol_) {
      phase_ = Phase::AlignGoal;
      return;
    }
    // Finish the final heading correction despite a small position change during a spin.
    // The goal checker still verifies XY and yaw together before accepting the goal.
    const double goal_exit = goal_dist_tol_ + costmap_->getResolution();
    if (phase_ == Phase::AlignGoal && goal_distance < goal_exit &&
      std::abs(goal_yaw) > goal_yaw_tolerance)
    {
      return;
    }
    updatePathPhase(path_heading, align_enter, align_exit);
  }

  void updatePathPhase(double path_heading, double align_enter, double align_exit)
  {
    if (!use_rotate_to_heading_) {
      phase_ = Phase::Tracking;
      return;
    }
    const double threshold = phase_ == Phase::AlignPath ? align_exit : align_enter;
    phase_ = std::abs(path_heading) > threshold ? Phase::AlignPath : Phase::Tracking;
  }

  void checkTrajectory(
    const geometry_msgs::msg::PoseStamped & pose, double distance,
    double linear, double lateral, double angular)
  {
    // Sample the actual commanded arc at no more than half a cell of footprint motion.
    const double radius = costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();
    const double speed = std::hypot(linear, lateral);
    const double edge_speed = speed + radius * std::abs(angular);
    double horizon = max_allowed_time_to_collision_up_to_carrot_;
    if (speed > 0.0) {
      horizon = std::min(horizon, distance / speed);
    }
    const auto steps = std::max(1, static_cast<int>(
        std::ceil(2.0 * horizon * edge_speed / costmap_->getResolution())));
    nav_msgs::msg::Path arc;
    arc.header = pose.header;
    const double yaw = tf2::getYaw(pose.pose.orientation);
    for (int step = 0; step <= steps; ++step) {
      const double t = horizon * step / steps;
      auto sample = pose;
      double heading = yaw;
      if (lateral != 0.0 && angular == 0.0) {
        const double dx_body = linear * t;
        const double dy_body = lateral * t;
        sample.pose.position.x += dx_body * std::cos(yaw) - dy_body * std::sin(yaw);
        sample.pose.position.y += dx_body * std::sin(yaw) + dy_body * std::cos(yaw);
      } else {
        const double half_turn = angular * t / 2.0;
        const double chord = linear * t *
          (half_turn == 0.0 ? 1.0 : std::sin(half_turn) / half_turn);
        sample.pose.position.x += chord * std::cos(yaw + half_turn);
        sample.pose.position.y += chord * std::sin(yaw + half_turn);
        heading = yaw + 2.0 * half_turn;
        sample.pose.orientation.z = std::sin(heading / 2.0);
        sample.pose.orientation.w = std::cos(heading / 2.0);
        sample.pose.orientation.x = sample.pose.orientation.y = 0.0;
      }
      arc.poses.push_back(sample);
      if (inCollision(sample.pose.position.x, sample.pose.position.y, heading)) {
        carrot_arc_pub_->publish(arc);
        throw nav2_core::PlannerException("DeltaPursuit predicted a collision on its feasible arc");
      }
    }
    carrot_arc_pub_->publish(arc);
  }

  double minimum_turn_radius_{0.5};
  pluginlib::ClassLoader<nav2_core::Controller> controller_loader_{"nav2_core", "nav2_core::Controller"};
  pluginlib::UniquePtr<nav2_core::Controller> tracking_controller_;
  double max_tracking_angular_speed_{0.0};
  double max_approach_linear_speed_{0.0};
  double terminal_settle_hold_time_{0.30};
  double terminal_settle_timeout_{3.0};
  Phase phase_{Phase::Tracking};
  geometry_msgs::msg::PoseStamped endpoint_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr endpoint_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr accepted_plan_pub_;
  Clock::time_point last_compute_;
  int64_t last_pose_stamp_{0};
  TerminalParking parking_;
  TerminalParking::Command last_parking_command_{TerminalParking::Command::FollowPath};
  bool parking_finished_{false};
  bool parking_failed_{false};
  std::string parking_failure_reason_;
  RepositionMode reposition_mode_{RepositionMode::Select};
  int reposition_switches_{0};
  int reposition_clearance_misses_{0};
  std::optional<Clock::time_point> reposition_path_started_;
  double reposition_path_min_dist_{std::numeric_limits<double>::infinity()};
  double min_approach_dist_{std::numeric_limits<double>::infinity()};
  double diag_dir_x_{0.0};
  double diag_dir_y_{0.0};
  geometry_msgs::msg::Point axial_start_;
  Cloud::ConstSharedPtr cloud_;
  struct CloudSample
  {
    Clock::time_point stamp;
    double x;
    double y;
    double yaw;
    Cloud::ConstSharedPtr cloud;
    std::string frame_id;
    tf2::Transform cloud_to_frame;
    bool has_transform{false};
  };
  std::deque<CloudSample> recent_clouds_;
  rclcpp::Subscription<Cloud>::SharedPtr cloud_sub_;
  struct FootprintSample
  {
    Clock::time_point stamp;
    double x;
    double y;
  };
  std::deque<FootprintSample> recent_footprints_;
};

}  // namespace odin_nav_sim

PLUGINLIB_EXPORT_CLASS(odin_nav_sim::DeltaPursuitController, nav2_core::Controller)
PLUGINLIB_EXPORT_CLASS(odin_nav_sim::SettledGoalChecker, nav2_core::GoalChecker)
