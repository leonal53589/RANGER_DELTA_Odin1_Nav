#ifndef ODIN_NAV_SIM__WAYPOINT_TURN_HPP_
#define ODIN_NAV_SIM__WAYPOINT_TURN_HPP_

#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

#include "nav_msgs/msg/path.hpp"

namespace odin_nav_sim
{

using Waypoints = std::vector<geometry_msgs::msg::PoseStamped>;

struct WaypointTurn
{
  std::size_t waypoint;
  double outgoing_yaw;
};

inline std::optional<WaypointTurn> findWaypointTurn(
  const nav_msgs::msg::Path & path, const Waypoints & goals,
  double turn_angle, double match_distance)
{
  constexpr double duplicate_distance = 1e-5;
  constexpr double tangent_distance = 0.20;
  constexpr double minimum_tangent = 0.025;
  const auto distance = [](const auto & a, const auto & b) {
      return std::hypot(a.x - b.x, a.y - b.y);
    };
  std::optional<WaypointTurn> result;
  double best_match = match_distance;
  const auto & poses = path.poses;
  for (std::size_t i = 1; i + 1 < poses.size(); ++i) {
    const auto & join = poses[i].pose.position;
    // ComputePathThroughPoses can repeat the junction as the start of the next path.
    if (distance(join, poses[i - 1].pose.position) <= duplicate_distance) {
      continue;
    }
    std::size_t before = i - 1;
    while (before > 0 && distance(join, poses[before].pose.position) < tangent_distance) {
      --before;
    }
    std::size_t after = i + 1;
    while (after + 1 < poses.size() &&
      distance(join, poses[after].pose.position) < tangent_distance)
    {
      ++after;
    }
    const auto & a = poses[before].pose.position;
    const auto & b = poses[after].pose.position;
    if (distance(a, join) < minimum_tangent || distance(b, join) < minimum_tangent) {
      continue;
    }
    const double incoming = std::atan2(join.y - a.y, join.x - a.x);
    const double outgoing = std::atan2(b.y - join.y, b.x - join.x);
    if (std::abs(std::remainder(outgoing - incoming, 2.0 * M_PI)) < turn_angle) {
      continue;
    }
    for (std::size_t goal = 0; goal + 1 < goals.size(); ++goal) {
      const double separation = distance(join, goals[goal].pose.position);
      if (separation <= match_distance &&
        (!result || goal < result->waypoint ||
        (goal == result->waypoint && separation < best_match)))
      {
        result = WaypointTurn{goal, outgoing};
        best_match = separation;
      }
    }
  }
  return result;
}

}  // namespace odin_nav_sim

#endif  // ODIN_NAV_SIM__WAYPOINT_TURN_HPP_
