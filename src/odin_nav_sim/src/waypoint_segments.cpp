#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/control_node.h"
#include "nav2_util/robot_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "waypoint_turn.hpp"

namespace odin_nav_sim
{

class WaypointSegments : public BT::ControlNode
{
public:
  WaypointSegments(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ControlNode(name, config),
    node_(config.blackboard->get<rclcpp::Node::SharedPtr>("node")),
    tf_(config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer")) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::BidirectionalPort<Waypoints>("goals"),
      BT::InputPort<nav_msgs::msg::Path>("preview"),
      BT::BidirectionalPort<Waypoints>("segment"),
      BT::InputPort<double>("turn_angle", 3.0 * M_PI / 4.0, "Minimum turn angle in radians"),
      BT::InputPort<double>("match_distance", 0.20, "Waypoint match tolerance in metres"),
    };
  }

  void halt() override
  {
    remaining_.clear();
    segment_size_ = 0;
    BT::ControlNode::halt();
  }

private:
  BT::NodeStatus tick() override
  {
    using Status = BT::NodeStatus;
    if (childrenCount() != 2) {
      throw BT::LogicError("WaypointSegments needs preview and tracking children");
    }
    auto goals = getInput<Waypoints>("goals");
    if (!goals || goals.value().empty()) {
      halt();
      return Status::FAILURE;
    }
    setStatus(Status::RUNNING);
    // A new NavigateThroughPoses request replaces the remaining goals on the blackboard.
    if (goals.value() != remaining_) {
      haltChildren();
      remaining_ = std::move(goals.value());
      segment_size_ = 0;
    }
    if (segment_size_ == 0) {
      if (remaining_.size() > 1) {
        const auto preview_status = children_nodes_[0]->executeTick();
        if (preview_status != Status::SUCCESS) {
          return preview_status;
        }
        haltChild(0);
      }
      if (!startSegment()) {
        return Status::FAILURE;
      }
    }

    const auto status = children_nodes_[1]->executeTick();
    const auto segment = getInput<Waypoints>("segment");
    if (!segment || segment.value().empty() || segment.value().size() > segment_size_) {
      halt();
      return Status::FAILURE;
    }
    if (status == Status::SUCCESS) {
      // By design, RemovePassedGoals leaves the active segment's endpoint for FollowPath.
      remaining_.erase(remaining_.begin(), remaining_.begin() + segment_size_);
      segment_size_ = 0;
      haltChildren();
      setOutput("goals", remaining_);
      return remaining_.empty() ? Status::SUCCESS : Status::RUNNING;
    }
    if (segment.value().size() < segment_size_) {
      const auto passed = segment_size_ - segment.value().size();
      remaining_.erase(remaining_.begin(), remaining_.begin() + passed);
      segment_size_ -= passed;
      setOutput("goals", remaining_);
    }
    if (status == Status::FAILURE) {
      halt();
    }
    return status;
  }

  bool startSegment()
  {
    auto segment = remaining_;
    if (segment.size() > 1) {
      const auto preview = getInput<nav_msgs::msg::Path>("preview");
      const auto turn_angle = getInput<double>("turn_angle");
      const auto match = getInput<double>("match_distance");
      if (!preview || preview.value().poses.empty() || !turn_angle || !match ||
        !std::isfinite(turn_angle.value()) || turn_angle.value() <= 0.0 ||
        turn_angle.value() > M_PI || !std::isfinite(match.value()) || match.value() <= 0.0)
      {
        RCLCPP_ERROR(node_->get_logger(), "Invalid preview path or turn detection settings");
        return false;
      }
      const auto & path = preview.value();
      auto map_goals = segment;
      for (auto & goal : map_goals) {
        if (goal.header.frame_id != path.header.frame_id) {
          geometry_msgs::msg::PoseStamped transformed;
          if (!nav2_util::transformPoseInTargetFrame(goal, transformed, *tf_, path.header.frame_id)) {
            RCLCPP_ERROR(node_->get_logger(), "Cannot transform waypoint to preview path frame");
            return false;
          }
          goal = transformed;
        }
      }
      const auto turn = findWaypointTurn(path, map_goals, turn_angle.value(), match.value());
      if (turn) {
        segment.resize(turn->waypoint + 1);
        auto stop = map_goals[turn->waypoint];
        tf2::Quaternion heading;
        heading.setRPY(0.0, 0.0, turn->outgoing_yaw);
        stop.pose.orientation = tf2::toMsg(heading);
        segment.back() = stop;
        RCLCPP_INFO(
          node_->get_logger(), "Stopping at waypoint (%.2f, %.2f) to align with the next segment",
          stop.pose.position.x, stop.pose.position.y);
      }
    }
    segment_size_ = segment.size();
    setOutput("segment", segment);
    return true;
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  Waypoints remaining_;
  std::size_t segment_size_{0};
};

}  // namespace odin_nav_sim

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<odin_nav_sim::WaypointSegments>("WaypointSegments");
}
