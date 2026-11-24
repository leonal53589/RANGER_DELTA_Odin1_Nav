#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/control_node.h"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace odin_nav_sim
{

using Pose = geometry_msgs::msg::PoseStamped;
using Path = nav_msgs::msg::Path;
using Clock = std::chrono::steady_clock;
using Status = BT::NodeStatus;

double separation(const Pose & a, const Pose & b)
{
  return std::hypot(a.pose.position.x - b.pose.position.x, a.pose.position.y - b.pose.position.y);
}

auto nearestPoint(const Path & path, const Pose & robot)
{
  return std::min_element(path.poses.begin(), path.poses.end(), [&](const Pose & a, const Pose & b) {
    return separation(a, robot) < separation(b, robot);
  });
}

double departure(const Path & path, const Pose & robot)
{
  auto ahead = nearestPoint(path, robot);
  while (ahead + 1 != path.poses.end() && separation(*ahead, robot) < 0.35) {
    ++ahead;
  }
  return std::atan2(ahead->pose.position.y - robot.pose.position.y,
    ahead->pose.position.x - robot.pose.position.x);
}

// Only bridge a remote blockage on a verified near stretch. A sparse or
// ambiguous plan must not skip the section actually under the chassis.
Path nearPrefix(const Path & path, const Pose & robot)
{
  Path prefix;
  prefix.header = path.header;
  auto it = nearestPoint(path, robot);
  if (it == path.poses.end() || separation(*it, robot) > 0.15) {
    return prefix;
  }
  // At an intersection FollowPath can still be tracking the other branch.
  bool left_neighborhood = false;
  bool visited_neighborhood = false;
  for (const auto & pose : path.poses) {
    const double d = separation(pose, robot);
    if (d <= 0.15) {
      if (left_neighborhood) {
        return prefix;
      }
      visited_neighborhood = true;
    } else if (visited_neighborhood && d > 0.30) {
      left_neighborhood = true;
    }
  }
  prefix.poses.push_back(robot);
  double length = 0.0;
  for (; it != path.poses.end(); ++it) {
    const double step = separation(*it, prefix.poses.back());
    if (step > 0.15) {
      break;
    }
    length += step;
    prefix.poses.push_back(*it);
    if (length >= 0.60) {
      return prefix;
    }
  }
  prefix.poses.clear();
  return prefix;
}

class ObstacleNavigation : public BT::ControlNode
{
public:
  ObstacleNavigation(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ControlNode(name, config),
    node_(config.blackboard->get<rclcpp::Node::SharedPtr>("node")),
    tf_(config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer")) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<Pose>("goal"), BT::InputPort<Path>("candidate"),
      BT::OutputPort<Path>("path"), BT::OutputPort<Path>("goal_probe"),
      BT::OutputPort<Path>("prefix"),
      BT::InputPort<double>("yield_seconds", 2.0, "Initial stopped wait"),
      BT::InputPort<double>("blocked_seconds", 20.0, "Continuous obstruction timeout")};
  }

  void halt() override
  {
    goal_.reset();
    blocked_.reset();
    handover_deadline_.reset();
    plan_started_.reset();
    alignment_credit_ = 0.0;
    active_.poses.clear();
    previous_heading_.reset();
    pending_heading_.reset();
    anchor_.reset();
    next_check_ = {};
    last_failure_ = Failure::Planning;
    BT::ControlNode::halt();
  }

private:
  // Planner, active path check, follower, goal check, near-prefix check.
  enum class Failure {Planning, Tracking};
  static constexpr auto period = std::chrono::milliseconds(500);

  void stopTracking(Clock::time_point now)
  {
    haltChild(0);
    plan_started_.reset();
    haltChild(2);
    handover_deadline_.reset();
    if (!blocked_) {
      blocked_ = now;
      alignment_credit_ = 0.0;
      RCLCPP_INFO(node_->get_logger(), "Route blocked: yielding before replanning");
      next_check_ = now + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(yield_));
    } else {
      next_check_ = now + period;
    }
    active_.poses.clear();
    anchor_.reset();
    pending_heading_.reset();
  }

  void checkProgress(Clock::time_point now, double heading, const Pose & map_pose)
  {
    Pose odom;
    if (!nav2_util::getCurrentPose(odom, *tf_, "odom", "base_link", 0.0)) {
      return;
    }
    if (!anchor_) {
      anchor_ = odom.pose.position;
      progress_heading_ = heading - tf2::getYaw(map_pose.pose.orientation) +
        tf2::getYaw(odom.pose.orientation);
      initial_yaw_error_ = std::abs(std::remainder(
        progress_heading_ - tf2::getYaw(odom.pose.orientation), 2.0 * M_PI));
      tracking_since_ = now;
    }
    // Count only a new best reduction towards this departure heading in wheel
    // coordinates. Oscillation and repeated short plans cannot accumulate credit.
    const double error = std::abs(std::remainder(
      progress_heading_ - tf2::getYaw(odom.pose.orientation), 2.0 * M_PI));
    const double earned = std::min(initial_yaw_error_ - error,
      0.30 * std::chrono::duration<double>(now - tracking_since_).count());
    alignment_credit_ = std::max(alignment_credit_, earned / 0.30);
    const auto & p = odom.pose.position;
    const double advance = (p.x - anchor_->x) * std::cos(progress_heading_) +
      (p.y - anchor_->y) * std::sin(progress_heading_);
    if (advance >= 0.12 && now - tracking_since_ >= std::chrono::milliseconds(800)) {
      blocked_.reset();
      anchor_.reset();
      RCLCPP_INFO(node_->get_logger(), "Route progress resumed on wheel odometry");
    }
  }

  bool validatePath(const Path & path)
  {
    if (path.poses.empty()) {
      return false;
    }
    setOutput("prefix", path);
    const auto valid = children_nodes_[4]->executeTick();
    haltChild(4);
    return valid == Status::SUCCESS;
  }

  bool beginHandover(const Pose & robot)
  {
    if (!validatePath(nearPrefix(active_, robot))) {
      return false;
    }
    handover_deadline_ = Clock::now() + std::chrono::milliseconds(250);
    return true;
  }

  void tryHandover(const Pose & robot)
  {
    if (Clock::now() < *handover_deadline_) {
      const auto result = children_nodes_[0]->executeTick();
      if (Clock::now() < *handover_deadline_) {
        if (result == Status::RUNNING) {
          return;
        }
        const auto path = getInput<Path>("candidate");
        if (result == Status::SUCCESS && path && !path.value().poses.empty() &&
          path.value().header.frame_id == active_.header.frame_id &&
          std::abs(std::remainder(departure(path.value(), robot) - departure(active_, robot),
          2.0 * M_PI)) <= M_PI / 6 && validatePath(path.value()) &&
          Clock::now() < *handover_deadline_)
        {
          haltChild(0);
          active_ = path.value();
          setOutput("path", active_);
          handover_deadline_.reset();
          next_check_ = Clock::now() + period;
          RCLCPP_INFO(node_->get_logger(), "Remote obstruction: switched to a same-direction route");
          return;
        }
      }
    }
    stopTracking(Clock::now());
  }

  Status tick() override
  {
    if (childrenCount() != 5) {
      throw BT::LogicError("ObstacleNavigation requires five children");
    }
    const auto goal = getInput<Pose>("goal");
    if (!goal || !getInput("yield_seconds", yield_) || !getInput("blocked_seconds", timeout_) ||
      !std::isfinite(yield_) || !std::isfinite(timeout_) || yield_ < 0 || timeout_ <= yield_)
    {
      halt();
      return Status::FAILURE;
    }
    if (!goal_ || *goal_ != goal.value()) {
      halt();
      goal_ = goal.value();
    }
    setStatus(Status::RUNNING);
    const auto now = Clock::now();

    if (!active_.poses.empty()) {
      Pose robot;
      const bool have_pose = nav2_util::getCurrentPose(robot, *tf_, active_.header.frame_id, "base_link", 0.0);
      if (have_pose) {
        previous_heading_ = departure(active_, robot);
        if (blocked_) {
          checkProgress(now, *previous_heading_, robot);
        }
      }
      if (!handover_deadline_ && now >= next_check_) {
        const auto valid = children_nodes_[1]->executeTick();
        haltChild(1);
        next_check_ = now + period;
        if (valid != Status::SUCCESS) {
          last_failure_ = Failure::Planning;
          if (blocked_ || !have_pose || !beginHandover(robot)) {
            stopTracking(now);
          }
        }
      }
      if (handover_deadline_) {
        if (have_pose) {
          tryHandover(robot);
        } else {
          stopTracking(now);
        }
      }
      if (!active_.poses.empty()) {
        const auto track = children_nodes_[2]->executeTick();
        if (track == Status::SUCCESS) {
          halt();
          return Status::SUCCESS;
        }
        if (track == Status::FAILURE) {
          last_failure_ = Failure::Tracking;
          stopTracking(now);
        }
      }
    }

    const double budget = timeout_ + (active_.poses.empty() ? 0.0 : alignment_credit_);
    if (blocked_ && std::chrono::duration<double>(now - *blocked_).count() >= budget) {
      haltChild(2);
      Path probe;
      probe.header = goal_->header;
      probe.poses.push_back(*goal_);
      setOutput("goal_probe", probe);
      const auto goal_valid = children_nodes_[3]->executeTick() == Status::SUCCESS;
      const char * reason = !goal_valid ? "goal cell could not be validated" :
        (last_failure_ == Failure::Planning ? "no sustained valid route" :
        "controller could not make sustained progress");
      RCLCPP_WARN(node_->get_logger(), "Blocked for %.1f s: %s", std::chrono::duration<double>(now - *blocked_).count(), reason);
      halt();
      return Status::FAILURE;
    }
    if (!active_.poses.empty() || now < next_check_) {
      return Status::RUNNING;
    }
    if (!plan_started_) {
      plan_started_ = now;
    }
    const auto plan = children_nodes_[0]->executeTick();
    if (plan == Status::RUNNING) {
      if (std::chrono::duration<double>(now - *plan_started_).count() >= timeout_) {
        blocked_ = *plan_started_;
        last_failure_ = Failure::Planning;
        stopTracking(now);
      }
      return Status::RUNNING;
    }
    plan_started_.reset();
    haltChild(0);
    const auto candidate = getInput<Path>("candidate");
    if (plan != Status::SUCCESS || !candidate || candidate.value().poses.empty()) {
      last_failure_ = Failure::Planning;
      stopTracking(now);
      return Status::RUNNING;
    }

    const auto & path = candidate.value();
    if (blocked_) {
      Pose robot;
      if (!nav2_util::getCurrentPose(robot, *tf_, path.header.frame_id, "base_link", 0.0)) {
        next_check_ = now + period;
        return Status::RUNNING;
      }
      const double heading = departure(path, robot);
      auto angle = [heading](double other) {
          return std::abs(std::remainder(heading - other, 2.0 * M_PI));
        };
      // Require two successive plans before committing to a large change of direction.
      if ((angle(tf2::getYaw(robot.pose.orientation)) > M_PI / 3 ||
        (previous_heading_ && angle(*previous_heading_) > M_PI / 3)) &&
        (!pending_heading_ || angle(*pending_heading_) > M_PI / 6))
      {
        pending_heading_ = heading;
        next_check_ = now + period;
        return Status::RUNNING;
      }
    }
    pending_heading_.reset();
    last_failure_ = Failure::Tracking;
    active_ = path;
    setOutput("path", active_);
    next_check_ = now + period;
    return Status::RUNNING;
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::optional<Pose> goal_;
  Path active_;
  std::optional<Clock::time_point> blocked_, handover_deadline_, plan_started_;
  double initial_yaw_error_{0.0}, alignment_credit_{0.0};
  Clock::time_point next_check_{}, tracking_since_{};
  std::optional<double> previous_heading_, pending_heading_;
  std::optional<geometry_msgs::msg::Point> anchor_;
  double progress_heading_{0.0}, yield_{2.0}, timeout_{20.0};
  Failure last_failure_{Failure::Planning};
};

}  // namespace odin_nav_sim

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<odin_nav_sim::ObstacleNavigation>("ObstacleNavigation");
}
