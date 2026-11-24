#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>

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

// Compare routes where the detour occurs, up to the global obstacle marking range.
// Equal arc-length samples avoid mistaking different planner point densities for a new route.
using RouteSamples = std::array<geometry_msgs::msg::Point, 14>;

RouteSamples sampleRoute(const Path & path, const Pose & robot)
{
  RouteSamples samples;
  auto it = nearestPoint(path, robot);
  double walked = 0.0;
  double target = 0.0;
  for (auto & point : samples) {
    target += 0.25;
    while (it + 1 != path.poses.end()) {
      const double step = separation(*it, *(it + 1));
      if (step > 0 && walked + step >= target) {
        break;
      }
      walked += step;
      ++it;
    }
    point = it->pose.position;
    if (it + 1 != path.poses.end()) {
      const auto & next = (it + 1)->pose.position;
      const double f = (target - walked) / separation(*it, *(it + 1));
      point.x += f * (next.x - point.x);
      point.y += f * (next.y - point.y);
    }
  }
  return samples;
}

bool similarRoute(const Path & a, const Path & b, const Pose & robot, double tolerance)
{
  if (a.poses.empty() || b.poses.empty() || a.header.frame_id != b.header.frame_id) {
    return false;
  }
  const auto first = sampleRoute(a, robot);
  const auto second = sampleRoute(b, robot);
  for (size_t i = 0; i < first.size(); ++i) {
    if (std::hypot(first[i].x - second[i].x, first[i].y - second[i].y) > tolerance) {
      return false;
    }
  }
  return true;
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
    pending_route_.poses.clear();
    previous_route_.poses.clear();
    tracking_failures_ = 0;
    retry_after_ = {};
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
    if (!active_.poses.empty()) {
      previous_route_ = std::move(active_);
      if (last_failure_ == Failure::Tracking) {
        ++tracking_failures_;
        if (tracking_failures_ >= 2) {
          retry_after_ = now + std::chrono::seconds(3);
          if (tracking_failures_ == 2) {
            RCLCPP_WARN(node_->get_logger(),
              "Controller repeatedly rejected this route without progress; spacing equivalent retries");
          }
        }
      } else {
        tracking_failures_ = 0;
      }
    }
    active_.poses.clear();
    anchor_.reset();
    pending_route_.poses.clear();
  }

  bool stalled(Clock::time_point now, double heading, const Pose & map_pose)
  {
    Pose odom;
    if (!nav2_util::getCurrentPose(odom, *tf_, "odom", "base_link", 0.0)) {
      return false;
    }
    if (!anchor_) {
      anchor_ = odom.pose.position;
      progress_heading_ = heading - tf2::getYaw(map_pose.pose.orientation) +
        tf2::getYaw(odom.pose.orientation);
      // Near the endpoint the controller turns to the requested final orientation.
      const auto & end = active_.poses.back();
      const double align_heading = separation(map_pose, end) <= 0.15 ?
        tf2::getYaw(end.pose.orientation) : heading;
      alignment_heading_ = align_heading - tf2::getYaw(map_pose.pose.orientation) +
        tf2::getYaw(odom.pose.orientation);
      initial_yaw_error_ = std::abs(std::remainder(
        alignment_heading_ - tf2::getYaw(odom.pose.orientation), 2.0 * M_PI));
      tracking_since_ = now;
    }
    // Count only a new best reduction towards this departure heading in wheel
    // coordinates. Oscillation and repeated short plans cannot accumulate credit.
    const double error = std::abs(std::remainder(
      alignment_heading_ - tf2::getYaw(odom.pose.orientation), 2.0 * M_PI));
    const double earned = std::min(initial_yaw_error_ - error,
      0.30 * std::chrono::duration<double>(now - tracking_since_).count());
    alignment_credit_ = std::max(alignment_credit_, earned / 0.30);
    const auto & p = odom.pose.position;
    const double advance = (p.x - anchor_->x) * std::cos(progress_heading_) +
      (p.y - anchor_->y) * std::sin(progress_heading_);
    if (advance >= (blocked_ ? 0.12 : 0.08) &&
      now - tracking_since_ >= std::chrono::milliseconds(800))
    {
      if (blocked_) {
        RCLCPP_INFO(node_->get_logger(), "Route progress resumed on wheel odometry");
      }
      tracking_failures_ = 0;
      retry_after_ = {};
      alignment_credit_ = 0.0;
      blocked_.reset();
      anchor_.reset();
      return false;
    }
    // A turn can earn at most pi / 0.30 s once. Rocking cannot keep extending the window.
    return std::chrono::duration<double>(now - tracking_since_).count() >= 12.0 + alignment_credit_;
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
          2.0 * M_PI)) <= M_PI / 6 && similarRoute(active_, path.value(), robot, 0.40) &&
          validatePath(path.value()) &&
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
        if (stalled(now, *previous_heading_, robot)) {
          RCLCPP_WARN(node_->get_logger(), "No net wheel progress along the route; yielding");
          last_failure_ = Failure::Tracking;
          stopTracking(now);
        }
      }
      if (!active_.poses.empty() && !handover_deadline_ && now >= next_check_) {
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
      const bool same_rejected_route = similarRoute(previous_route_, path, robot, 0.15);
      if (now < retry_after_ && same_rejected_route) {
        next_check_ = retry_after_;
        pending_route_.poses.clear();
        return Status::RUNNING;
      }
      // A shared initial tangent can lead to opposite sides of the obstruction.
      if ((angle(tf2::getYaw(robot.pose.orientation)) > M_PI / 3 ||
        (previous_heading_ && angle(*previous_heading_) > M_PI / 3) ||
        (!previous_route_.poses.empty() && !similarRoute(previous_route_, path, robot, 0.40))) &&
        (!similarRoute(pending_route_, path, robot, 0.25) ||
        angle(departure(pending_route_, robot)) > M_PI / 6))
      {
        pending_route_ = path;
        next_check_ = now + period;
        return Status::RUNNING;
      }
      if (!same_rejected_route) {
        tracking_failures_ = 0;
        retry_after_ = {};
      }
    }
    pending_route_.poses.clear();
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
  std::optional<double> previous_heading_;
  Path previous_route_, pending_route_;
  unsigned int tracking_failures_{0};
  Clock::time_point retry_after_{};
  double alignment_heading_{0.0};
  std::optional<geometry_msgs::msg::Point> anchor_;
  double progress_heading_{0.0}, yield_{2.0}, timeout_{20.0};
  Failure last_failure_{Failure::Planning};
};

}  // namespace odin_nav_sim

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<odin_nav_sim::ObstacleNavigation>("ObstacleNavigation");
}
