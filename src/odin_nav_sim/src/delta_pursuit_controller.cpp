#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <utility>

#include "angles/angles.h"
#include "nav2_controller/plugins/simple_goal_checker.hpp"
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

class SettledGoalChecker : public nav2_controller::SimpleGoalChecker
{
public:
  void reset() override
  {
    SimpleGoalChecker::reset();
    reached_ = false;
  }

  bool isGoalReached(
    const geometry_msgs::msg::Pose &, const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::Twist &) override
  {
    // Humble calls this after computeVelocityCommands, with an independently transformed goal.
    return std::exchange(reached_, false);
  }

  void setReached(bool reached) {reached_ = reached;}

private:
  bool reached_{false};
};

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
      parking_ = {};
      parking_finished_ = false;
      reposition_mode_ = RepositionMode::Select;
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
    geometry_msgs::msg::Pose pose_tolerance;
    geometry_msgs::msg::Twist velocity_tolerance;
    double goal_yaw_tolerance = 0.10;
    if (goal_checker->getTolerances(pose_tolerance, velocity_tolerance)) {
      goal_dist_tol_ = pose_tolerance.position.x;
      goal_yaw_tolerance = std::abs(tf2::getYaw(pose_tolerance.orientation));
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
    std::optional<double> axial_velocity;
    if (settled_checker) {
      const auto parking = updateParking(pose, speed, goal_yaw_tolerance);
      parking_command = parking.command;
      goal_yaw = parking.heading_error;
      if (!parking_.isRepositioning()) {
        reposition_mode_ = RepositionMode::Select;
      }
      if (parking_command != last_parking_command_) {
        if (parking_command == Command::Reposition) {
          RCLCPP_INFO(logger_, "Terminal parking: repositioning toward the path endpoint");
        } else if (parking_command == Command::Reached) {
          RCLCPP_INFO(logger_, "Terminal parking: position and heading verified at rest");
        }
        last_parking_command_ = parking_command;
      }
      if (parking_command == Command::Failed) {
        parking_finished_ = true;
        throw nav2_core::PlannerException("Terminal parking could not settle within its limits");
      }
      if (parking_command == Command::Reposition) {
        axial_velocity = axialReposition(pose, parking, goal_yaw_tolerance);
      }
      if (!axial_velocity &&
        (parking_command == Command::FollowPath || parking_command == Command::Reposition))
      {
        updatePathPhase(path_heading, align_enter, align_exit);
      } else {
        phase_ = Phase::Tracking;
      }
    } else {
      updatePhase(
        std::hypot(end.position.x, end.position.y), goal_yaw, goal_yaw_tolerance,
        path_heading, align_enter, align_exit);
    }

    double linear = desired_linear_vel_;
    double angular = 0.0;

    if (axial_velocity) {
      linear = *axial_velocity;
    } else if (parking_command == Command::Hold || parking_command == Command::Reached) {
      linear = 0.0;
    } else if (parking_command == Command::AlignGoal || phase_ == Phase::AlignGoal) {
      rotateToHeading(linear, angular, goal_yaw, speed);
      if (settled_checker) {
        // Proportional speed near the final heading, in addition to the RPP limits.
        angular = std::clamp(angular, -std::abs(goal_yaw), std::abs(goal_yaw));
      }
    } else if (phase_ == Phase::AlignPath) {
      rotateToHeading(linear, angular, path_heading, speed);
    } else if (tracking_controller_ && parking_command == Command::FollowPath) {
      const auto tracking = tracking_controller_->computeVelocityCommands(pose, speed, goal_checker);
      if (!std::isfinite(tracking.twist.linear.x) || !std::isfinite(tracking.twist.angular.z)) {
        throw nav2_core::PlannerException("Tracking controller returned a non-finite velocity");
      }
      double speed_limit = std::abs(desired_linear_vel_);
      if (max_approach_linear_speed_ > 0.0 && approachVelocityScalingFactor(path) < 1.0) {
        speed_limit = std::min(speed_limit, max_approach_linear_speed_);
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
        linear = 0.0;
        angular = 0.0;
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
      checkTrajectory(pose, distance, linear, angular);
    }
    if (parking_command == Command::Reached) {
      parking_finished_ = true;
      settled_checker->setReached(true);
    }
    geometry_msgs::msg::TwistStamped command;
    command.header = pose.header;
    command.twist.linear.x = linear;
    command.twist.angular.z = angular;
    return command;
  }

private:
  using Clock = std::chrono::steady_clock;
  using Cloud = sensor_msgs::msg::PointCloud2;
  enum class Phase {Tracking, AlignPath, AlignGoal};
  enum class RepositionMode {Select, Path, Forward, Reverse};

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
    return {parking_.update({time, std::hypot(dx, dy), yaw, goal_dist_tol_, goal_yaw_tolerance,
        stopped, fresh}), yaw, longitudinal, lateral};
  }

  std::optional<double> axialReposition(
    const geometry_msgs::msg::PoseStamped & pose, const ParkingUpdate & parking,
    double yaw_tolerance)
  {
    if (reposition_mode_ == RepositionMode::Path) {
      return std::nullopt;
    }
    const auto sweep = axialSweepDistance(
      parking.longitudinal, parking.lateral, kRepositionToleranceFactor * goal_dist_tol_);
    const bool selecting = reposition_mode_ == RepositionMode::Select;
    const auto & position = pose.pose.position;
    const double traveled = selecting ? 0.0 :
      std::hypot(position.x - axial_start_.x, position.y - axial_start_.y);
    const auto direction = parking.longitudinal < 0.0 ?
      RepositionMode::Reverse : RepositionMode::Forward;
    const bool geometry_ok = sweep && std::abs(parking.heading_error) < yaw_tolerance &&
      traveled + std::abs(*sweep) <= kMaxAxialTravel &&
      (selecting || direction == reposition_mode_);
    if (!geometry_ok || !use_collision_detection_ || !hasObservedClearance(pose, *sweep)) {
      if (!selecting) {
        parking_finished_ = true;
        throw nav2_core::PlannerException("Terminal parking lost straight-move clearance");
      }
      reposition_mode_ = RepositionMode::Path;
      RCLCPP_INFO(
        logger_, "Terminal parking: following path for repositioning (dx=%.3f, dy=%.3f; %s)",
        parking.longitudinal, parking.lateral,
        geometry_ok ? "no verified straight clearance" : "outside straight-move limits");
      return std::nullopt;
    }
    if (selecting) {
      reposition_mode_ = direction;
      axial_start_ = position;
      RCLCPP_INFO(
        logger_, "Terminal parking: straight %s correction (dx=%.3f, dy=%.3f)",
        direction == RepositionMode::Reverse ? "reverse" : "forward",
        parking.longitudinal, parking.lateral);
    }
    return std::copysign(std::min(kAxialSpeed, std::abs(desired_linear_vel_)), *sweep);
  }

  bool hasObservedClearance(const geometry_msgs::msg::PoseStamped & pose, double sweep) const
  {
    const auto cloud = std::atomic_load(&cloud_);
    if (!cloud || cloud->data.empty() || cloud->header.frame_id.empty() || cloud->is_bigendian ||
      cloud->height == 0 || cloud->width == 0 || cloud->point_step == 0 ||
      static_cast<size_t>(cloud->width) * cloud->point_step != cloud->row_step ||
      cloud->data.size() != static_cast<size_t>(cloud->row_step) * cloud->height)
    {
      return false;
    }
    const auto stamp = rclcpp::Time(cloud->header.stamp);
    const auto age = (clock_->now() - stamp).seconds();
    if (stamp.nanoseconds() == 0 || age < 0.0 || age > 0.35) {
      return false;
    }
    const auto field_offset = [&cloud](const char * name) -> std::optional<size_t> {
        const auto field = std::find_if(
          cloud->fields.begin(), cloud->fields.end(),
          [name](const auto & f) {return f.name == name;});
        if (field == cloud->fields.end() || field->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
          field->count != 1 || field->offset + sizeof(float) > cloud->point_step)
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
    try {
      const auto transform = tf_->lookupTransform(
        pose.header.frame_id, cloud->header.frame_id, stamp, rclcpp::Duration::from_seconds(0.0));
      tf2::fromMsg(transform.transform, cloud_to_odom);
    } catch (const tf2::TransformException &) {
      return false;
    }
    TerminalClearance clearance(*costmap_);
    const auto & origin = cloud_to_odom.getOrigin();
    const auto * data = cloud->data.data();
    for (size_t i = 0; i < cloud->data.size(); i += cloud->point_step) {
      // ODIN1 packs points at a 19-byte stride, so XYZ need not be aligned.
      float x, y, z;
      std::memcpy(&x, data + i + *x_offset, sizeof(float));
      std::memcpy(&y, data + i + *y_offset, sizeof(float));
      std::memcpy(&z, data + i + *z_offset, sizeof(float));
      if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
        clearance.observe(origin, cloud_to_odom * tf2::Vector3(x, y, z));
      }
    }
    return clearance.permits(
      *costmap_, pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation),
      sweep, costmap_ros_->getLayeredCostmap()->getCircumscribedRadius());
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
    double linear, double angular)
  {
    // Sample the actual commanded arc at no more than half a cell of footprint motion.
    const double radius = costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();
    const double edge_speed = std::abs(linear) + radius * std::abs(angular);
    double horizon = max_allowed_time_to_collision_up_to_carrot_;
    if (std::abs(linear) > 0.0) {
      horizon = std::min(horizon, distance / std::abs(linear));
    }
    const auto steps = std::max(1, static_cast<int>(
        std::ceil(2.0 * horizon * edge_speed / costmap_->getResolution())));
    nav_msgs::msg::Path arc;
    arc.header = pose.header;
    const double yaw = tf2::getYaw(pose.pose.orientation);
    for (int step = 0; step <= steps; ++step) {
      const double t = horizon * step / steps;
      const double half_turn = angular * t / 2.0;
      const double chord = linear * t *
        (half_turn == 0.0 ? 1.0 : std::sin(half_turn) / half_turn);
      auto sample = pose;
      sample.pose.position.x += chord * std::cos(yaw + half_turn);
      sample.pose.position.y += chord * std::sin(yaw + half_turn);
      const double heading = yaw + 2.0 * half_turn;
      sample.pose.orientation.z = std::sin(heading / 2.0);
      sample.pose.orientation.w = std::cos(heading / 2.0);
      sample.pose.orientation.x = sample.pose.orientation.y = 0.0;
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
  Phase phase_{Phase::Tracking};
  geometry_msgs::msg::PoseStamped endpoint_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr endpoint_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr accepted_plan_pub_;
  Clock::time_point last_compute_;
  int64_t last_pose_stamp_{0};
  TerminalParking parking_;
  TerminalParking::Command last_parking_command_{TerminalParking::Command::FollowPath};
  bool parking_finished_{false};
  RepositionMode reposition_mode_{RepositionMode::Select};
  geometry_msgs::msg::Point axial_start_;
  Cloud::ConstSharedPtr cloud_;
  rclcpp::Subscription<Cloud>::SharedPtr cloud_sub_;
};

}  // namespace odin_nav_sim

PLUGINLIB_EXPORT_CLASS(odin_nav_sim::DeltaPursuitController, nav2_core::Controller)
PLUGINLIB_EXPORT_CLASS(odin_nav_sim::SettledGoalChecker, nav2_core::GoalChecker)
