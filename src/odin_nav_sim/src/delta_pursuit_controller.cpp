#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "nav2_core/exceptions.hpp"
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "pluginlib/class_list_macros.hpp"

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
    const auto parameter = name + ".minimum_turn_radius";
    if (!node->has_parameter(parameter)) {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.read_only = true;
      node->declare_parameter(parameter, 0.5, descriptor);
    }
    minimum_turn_radius_ = node->get_parameter(parameter).as_double();
    if (!std::isfinite(minimum_turn_radius_) || minimum_turn_radius_ <= 0.0) {
      throw std::invalid_argument("minimum_turn_radius must be finite and positive");
    }
  }

  void setPlan(const nav_msgs::msg::Path & path) override
  {
    RegulatedPurePursuitController::setPlan(path);
    std::lock_guard<std::mutex> controller_lock(mutex_);
    phase_ = Phase::Tracking;
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & speed,
    nav2_core::GoalChecker * goal_checker) override
  {
    std::lock_guard<std::mutex> controller_lock(mutex_);
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*costmap_->getMutex());
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
    const auto & end = path.poses.back().pose;
    const double goal_distance = std::hypot(end.position.x, end.position.y);
    const double goal_yaw = tf2::getYaw(end.orientation);
    const double path_heading = std::atan2(point.y, point.x);
    const double curvature_limit = 1.0 / minimum_turn_radius_;
    const double feasible_heading = std::asin(std::min(1.0, distance * curvature_limit / 2.0));
    const double align_enter = std::min(rotate_to_heading_min_angle_,
        std::max(0.18, feasible_heading));
    const double align_exit = std::min(0.15, 0.55 * align_enter);
    updatePhase(goal_distance, goal_yaw, goal_yaw_tolerance, path_heading, align_enter, align_exit);
    double linear = desired_linear_vel_;
    double angular = 0.0;

    if (phase_ == Phase::AlignGoal) {
      rotateToHeading(linear, angular, goal_yaw, speed);
    } else if (phase_ == Phase::AlignPath) {
      rotateToHeading(linear, angular, path_heading, speed);
    } else {
      const double curvature = distance > 0.0 ? 2.0 * point.y / (distance * distance) : 0.0;
      double direction = allow_reversing_ && point.x < 0.0 ? -1.0 : 1.0;
      applyConstraints(
        curvature, speed, costAtPose(pose.pose.position.x, pose.pose.position.y), path,
        linear, direction);
      angular = linear * std::clamp(curvature, -curvature_limit, curvature_limit);
    }

    if (use_collision_detection_) {
      checkTrajectory(pose, distance, linear, angular);
    }
    geometry_msgs::msg::TwistStamped command;
    command.header = pose.header;
    command.twist.linear.x = linear;
    command.twist.angular.z = angular;
    return command;
  }

private:
  enum class Phase {Tracking, AlignPath, AlignGoal};

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
  Phase phase_{Phase::Tracking};
};

}  // namespace odin_nav_sim

PLUGINLIB_EXPORT_CLASS(odin_nav_sim::DeltaPursuitController, nav2_core::Controller)
