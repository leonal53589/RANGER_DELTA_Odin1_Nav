#include <initializer_list>
#include <utility>

#include "gtest/gtest.h"
#include "../src/waypoint_turn.hpp"

namespace
{
using odin_nav_sim::findWaypointTurn;
using odin_nav_sim::Waypoints;

Waypoints poses(std::initializer_list<std::pair<double, double>> points)
{
  Waypoints result;
  for (auto [x, y] : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    result.push_back(pose);
  }
  return result;
}

TEST(WaypointTurn, DetectsRecordedFoldbackWithDuplicateJoin)
{
  nav_msgs::msg::Path path;
  path.poses = poses({{1.175, 1.025}, {1.65, 0.98}, {1.875, 0.925},
      {1.875, 0.925}, {1.65, 0.98}, {1.45, 1.03}, {0.071, 0.186}});
  const auto goals = poses({{1.900, 0.929}, {0.071, 0.186}});
  auto turn = findWaypointTurn(path, goals, 3.0 * M_PI / 4.0, 0.20);
  ASSERT_TRUE(turn.has_value());
  EXPECT_EQ(turn->waypoint, 0u);
  EXPECT_NEAR(turn->outgoing_yaw, std::atan2(0.055, -0.225), 1e-6);

  path.poses.erase(path.poses.begin() + 3);
  turn = findWaypointTurn(path, goals, 3.0 * M_PI / 4.0, 0.20);
  ASSERT_TRUE(turn.has_value());
  EXPECT_EQ(turn->waypoint, 0u);
}

TEST(WaypointTurn, PreservesContinuousCorner)
{
  nav_msgs::msg::Path path;
  path.poses = poses({{0.0, 0.0}, {0.5, 0.0}, {1.0, 0.1}, {1.0, 0.1},
      {1.3, 0.3}, {1.6, 0.7}, {1.65, 1.0}});
  EXPECT_FALSE(findWaypointTurn(path, poses({{1.0, 0.1}, {1.65, 1.0}}),
    3.0 * M_PI / 4.0, 0.20));
}

TEST(WaypointTurn, RequiresAnIntermediateWaypointAtTheCorner)
{
  nav_msgs::msg::Path path;
  path.poses = poses({{0.0, 0.0}, {1.0, 0.0}, {0.8, 0.0}, {0.0, 0.0}});
  EXPECT_FALSE(findWaypointTurn(path, poses({{0.5, 0.0}, {0.0, 0.0}}),
    3.0 * M_PI / 4.0, 0.20));
  EXPECT_FALSE(findWaypointTurn(path, poses({{0.0, 0.0}}), 3.0 * M_PI / 4.0, 0.20));
}

}  // namespace
