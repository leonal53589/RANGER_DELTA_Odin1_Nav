#include <gtest/gtest.h>

#include "../src/terminal_clearance.hpp"
#include "../src/terminal_parking.hpp"

namespace odin_nav_sim
{
using Command = TerminalParking::Command;

TEST(TerminalParkingTest, BrakesBeforeCorrectingAnOffsetSeenDuringTheFinalTurn)
{
  TerminalParking parking;
  EXPECT_EQ(parking.update({0.0, 0.20, 0.6, 0.10, 0.20, false, true}), Command::FollowPath);
  EXPECT_EQ(parking.update({0.1, 0.0995, -0.61, 0.10, 0.20, false, true}), Command::AlignGoal);
  EXPECT_EQ(parking.update({1.0, 0.1243, -0.19, 0.10, 0.20, false, true}), Command::Hold);
  EXPECT_EQ(parking.update({1.2, 0.1243, -0.14, 0.10, 0.20, true, true}), Command::Hold);
  EXPECT_EQ(parking.update({1.4, 0.1243, -0.14, 0.10, 0.20, true, true}), Command::Hold);
  EXPECT_EQ(parking.update({1.6, 0.1243, -0.14, 0.10, 0.20, true, true}), Command::Hold);
  EXPECT_EQ(parking.update({1.7, 0.099, -0.14, 0.10, 0.20, true, true}), Command::Reposition);
  EXPECT_EQ(parking.update({2.7, 0.064, -0.30, 0.10, 0.20, false, true}), Command::AlignGoal);
  EXPECT_EQ(parking.update({3.1, 0.085, -0.19, 0.10, 0.20, false, true}), Command::Hold);
  EXPECT_EQ(parking.update({3.2, 0.085, -0.13, 0.10, 0.20, true, true}), Command::Hold);
  EXPECT_EQ(parking.update({3.6, 0.085, -0.13, 0.10, 0.20, true, true}), Command::Reached);
}

TEST(TerminalParkingTest, RequiresFreshPoseAndBothTolerancesThroughoutSettling)
{
  TerminalParking parking;
  EXPECT_EQ(parking.update({0.0, 0.08, 0.1, 0.10, 0.20, true, true}), Command::Hold);
  EXPECT_EQ(parking.update({0.2, 0.08, 0.1, 0.10, 0.20, true, false}), Command::Hold);
  EXPECT_EQ(parking.update({0.4, 0.08, 0.1, 0.10, 0.20, true, true}), Command::Hold);
  EXPECT_EQ(parking.update({0.5, 0.102, 0.1, 0.10, 0.20, true, true}), Command::Hold);
  EXPECT_EQ(parking.update({0.6, 0.08, 0.1, 0.10, 0.20, true, true}), Command::Hold);
  EXPECT_EQ(parking.update({1.0, 0.08, 0.1, 0.10, 0.20, true, true}), Command::Reached);
}

TEST(TerminalParkingTest, BoundsRepeatedPositionCorrections)
{
  TerminalParking parking;
  EXPECT_EQ(parking.update({0.0, 0.099, 0.6, 0.10, 0.20, true, true}), Command::AlignGoal);
  for (int trial = 0; trial < 2; ++trial) {
    const double t = 1.0 + trial * 2.0;
    EXPECT_EQ(parking.update({t, 0.12, 0.19, 0.10, 0.20, true, true}), Command::Hold);
    EXPECT_EQ(parking.update({t + 0.4, 0.12, 0.19, 0.10, 0.20, true, true}), Command::Hold);
    EXPECT_EQ(
      parking.update({t + 0.5, 0.11, 0.19, 0.10, 0.20, true, true}), Command::Reposition);
    EXPECT_EQ(parking.update({t + 1.0, 0.06, 0.6, 0.10, 0.20, true, true}), Command::AlignGoal);
  }
  EXPECT_EQ(parking.update({5.0, 0.12, 0.19, 0.10, 0.20, true, true}), Command::Hold);
  EXPECT_EQ(parking.update({5.4, 0.12, 0.19, 0.10, 0.20, true, true}), Command::Failed);
}

TEST(TerminalParkingTest, TimesOutWhenAStopCannotBeConfirmed)
{
  TerminalParking parking;
  EXPECT_EQ(parking.update({0.0, 0.09, 0.1, 0.10, 0.20, false, true}), Command::Hold);
  EXPECT_EQ(parking.update({2.0, 0.09, 0.1, 0.10, 0.20, false, true}), Command::Hold);
  EXPECT_EQ(parking.update({3.1, 0.09, 0.1, 0.10, 0.20, false, true}), Command::Failed);
}

TEST(TerminalParkingTest, ComputesShortestAxialMoveWithRoomToStop)
{
  const auto forward = axialSweepDistance(0.12, 0.02, 0.065);
  const auto reverse = axialSweepDistance(-0.12, -0.02, 0.065);
  ASSERT_TRUE(forward && reverse);
  EXPECT_NEAR(*forward, 0.08315, 0.0001);
  EXPECT_DOUBLE_EQ(*forward, -*reverse);
  EXPECT_FALSE(axialSweepDistance(-0.12, 0.07, 0.065));
  EXPECT_FALSE(axialSweepDistance(-0.26, 0.02, 0.065));
}

TEST(TerminalParkingTest, StraightSweepNeedsFreshClearEvidenceAlongItsDirection)
{
  using nav2_costmap_2d::FREE_SPACE;
  using nav2_costmap_2d::LETHAL_OBSTACLE;
  nav2_costmap_2d::Costmap2D map(80, 80, 0.05, -2.0, -2.0, FREE_SPACE);
  TerminalClearance clearance(map);
  const tf2::Vector3 origin(0, 0, 0.8);
  EXPECT_FALSE(clearance.permits(map, 0, 0, 0, -0.10, 0.30));

  for (double angle = -1.57; angle <= 1.57; angle += 0.003) {
    clearance.observe(origin, tf2::Vector3(std::cos(angle), std::sin(angle), 0.4));
  }
  EXPECT_TRUE(clearance.permits(map, 0, 0, 0, 0.10, 0.30));
  EXPECT_FALSE(clearance.permits(map, 0, 0, 0, -0.10, 0.30));

  for (double angle = -1.57; angle <= 1.57; angle += 0.003) {
    clearance.observe(origin, tf2::Vector3(-std::cos(angle), std::sin(angle), 0.4));
  }
  EXPECT_TRUE(clearance.permits(map, 0, 0, 0, -0.10, 0.30));
  unsigned int x, y;
  ASSERT_TRUE(map.worldToMap(-0.39, 0.0, x, y));
  map.setCost(x, y, LETHAL_OBSTACLE);
  EXPECT_FALSE(clearance.permits(map, 0, 0, 0, -0.10, 0.30));
  EXPECT_TRUE(clearance.permits(map, 0, 0, 0, 0.10, 0.30));
  clearance.observe(origin, tf2::Vector3(0.38, 0, 0.12));
  EXPECT_FALSE(clearance.permits(map, 0, 0, 0, 0.10, 0.30));
}
}  // namespace odin_nav_sim
