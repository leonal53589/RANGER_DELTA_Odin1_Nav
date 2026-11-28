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
  EXPECT_EQ(parking.update({1.7, 0.099, -0.24, 0.10, 0.20, true, true}), Command::Reposition);
  EXPECT_EQ(parking.update({2.7, 0.064, -0.30, 0.10, 0.20, false, true}), Command::AlignGoal);
  EXPECT_EQ(parking.update({3.1, 0.085, -0.19, 0.10, 0.20, false, true}), Command::Hold);
  EXPECT_EQ(parking.update({3.2, 0.085, -0.13, 0.10, 0.20, true, true}), Command::Hold);
  EXPECT_EQ(parking.update({3.6, 0.085, -0.13, 0.10, 0.20, true, true}), Command::Reached);
}

TEST(TerminalParkingTest, BrakesInsideBothTolerancesEvenOutsideInnerCorrectionMargin)
{
  TerminalParking parking(0.55, 4.0);
  EXPECT_EQ(parking.update({0.0, .020, .20, .022, .05, true, true}), Command::AlignGoal);
  EXPECT_EQ(parking.update({1.0, .027, .02, .022, .05, true, true}), Command::Hold);
  EXPECT_EQ(parking.update({1.6, .027, .02, .022, .05, true, true}), Command::Hold);
  EXPECT_EQ(parking.update({1.7, .025, .02, .022, .05, true, true}), Command::Reposition);
  EXPECT_EQ(parking.update({2.0, .021, .02, .022, .05, false, true}), Command::Hold);
  EXPECT_EQ(parking.update({2.1, .021, .02, .022, .05, true, true}), Command::Hold);
  EXPECT_EQ(parking.update({2.5, .021, .02, .022, .05, true, true}), Command::Hold);
  EXPECT_EQ(parking.update({2.7, .021, .02, .022, .05, true, true}), Command::Reached);
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

TEST(TerminalParkingTest, HonorsExtendedSettleHoldWindow)
{
  TerminalParking parking(0.55, 4.0);
  EXPECT_EQ(parking.update({0.0, 0.02, 0.02, 0.03, 0.06, true, true}), Command::Hold);
  EXPECT_EQ(parking.update({0.35, 0.02, 0.02, 0.03, 0.06, true, true}), Command::Hold);
  EXPECT_EQ(parking.update({0.60, 0.02, 0.02, 0.03, 0.06, true, true}), Command::Reached);
}

TEST(TerminalParkingTest, PermitsShortReverseOverRecentlyOccupiedFootprintUnlessBlocked)
{
  using nav2_costmap_2d::FREE_SPACE;
  using nav2_costmap_2d::LETHAL_OBSTACLE;
  nav2_costmap_2d::Costmap2D map(80, 80, 0.05, -2.0, -2.0, FREE_SPACE);
  TerminalClearance clearance(map);
  EXPECT_FALSE(clearance.permits(map, 0, 0, 0, -0.06, 0.30));

  // Robot drove forward from x=-0.15 to x=0.0, covering the reverse sweep with its recent footprint.
  for (double hx = -0.15; hx <= 0.0; hx += 0.03) {
    clearance.markRecentlyOccupied(hx, 0.0, 0.30);
  }
  EXPECT_TRUE(clearance.permits(map, 0, 0, 0, -0.06, 0.30));

  unsigned int x, y;
  ASSERT_TRUE(map.worldToMap(-0.34, 0.0, x, y));
  map.setCost(x, y, LETHAL_OBSTACLE);
  EXPECT_FALSE(clearance.permits(map, 0, 0, 0, -0.06, 0.30));
}

TEST(TerminalParkingTest, ObserveFreeOnlySupportsLateralCrabClearanceWhilePreservingLiveObstacles)
{
  using nav2_costmap_2d::FREE_SPACE;
  nav2_costmap_2d::Costmap2D map(80, 80, 0.05, -2.0, -2.0, FREE_SPACE);
  TerminalClearance clearance(map);
  const double right_yaw = M_PI_2;  // body +Y (left/right lateral sweep axis)

  // Robot previously observed the sideways area from 0.40 m back while pulling in,
  // even with a dynamic hit at (-0.2, -0.5) that has since walked away.
  const tf2::Vector3 prior_origin(-0.40, 0.0, 0.8);
  for (double angle = -1.57; angle <= 1.57; angle += 0.005) {
    clearance.observeFreeOnly(
      prior_origin,
      tf2::Vector3(-0.40 + 1.4 * std::cos(angle), 1.4 * std::sin(angle), 0.4));
  }
  clearance.observeFreeOnly(prior_origin, tf2::Vector3(0.0, -0.36, 0.4));

  // Lateral right shift (-0.07 along +Y -> -0.07 in Y) is permitted by prior free evidence.
  EXPECT_TRUE(clearance.permits(map, 0.0, 0.0, right_yaw, -0.07, 0.30));

  // A live return on the right side still immediately blocks lateral CRAB motion.
  const tf2::Vector3 live_origin(0.0, 0.0, 0.8);
  clearance.observe(live_origin, tf2::Vector3(0.0, -0.36, 0.4));
  EXPECT_FALSE(clearance.permits(map, 0.0, 0.0, right_yaw, -0.07, 0.30));
}

TEST(TerminalParkingTest, RelaxesInnerMarginForSmallResidualYawOnly)
{
  TerminalParking parking(0.55, 4.0);
  EXPECT_DOUBLE_EQ(TerminalParking::repositionToleranceFactor(0.06, 0.05), 0.85);
  EXPECT_DOUBLE_EQ(TerminalParking::repositionToleranceFactor(0.20, 0.05), kRepositionToleranceFactor);

  EXPECT_EQ(parking.update({0.0, 0.020, 0.01, 0.022, 0.05, true, true}), Command::Hold);
  EXPECT_EQ(parking.update({0.6, 0.025, 0.06, 0.022, 0.05, true, true}), Command::Hold);
  EXPECT_EQ(parking.update({1.2, 0.025, 0.06, 0.022, 0.05, true, true}), Command::Hold);
  EXPECT_EQ(parking.update({1.3, 0.025, 0.06, 0.022, 0.05, true, true}), Command::Reposition);
  // With small residual yaw (0.06 < 0.14), 0.018 <= 0.85 * 0.022 (0.0187) exits directly to Align.
  EXPECT_EQ(parking.update({1.5, 0.018, 0.06, 0.022, 0.05, false, true}), Command::AlignGoal);
}

TEST(TerminalParkingTest, PermitsDiagonalCrabSweepInObservedFreeSpace)
{
  using nav2_costmap_2d::FREE_SPACE;
  nav2_costmap_2d::Costmap2D map(80, 80, 0.05, -2.0, -2.0, FREE_SPACE);
  TerminalClearance clearance(map);
  const tf2::Vector3 prior_origin(-0.40, 0.0, 0.8);
  const tf2::Vector3 origin(0.0, 0.0, 0.8);
  for (double angle = -1.57; angle <= 1.57; angle += 0.005) {
    clearance.observeFreeOnly(
      prior_origin, tf2::Vector3(-0.40 + 1.4 * std::cos(angle), 1.4 * std::sin(angle), 0.4));
    clearance.observe(origin, tf2::Vector3(1.2 * std::cos(angle), 1.2 * std::sin(angle), 0.4));
  }
  EXPECT_TRUE(clearance.permits(map, 0.0, 0.0, -M_PI_4, 0.08, 0.30));
  clearance.observe(origin, tf2::Vector3(0.28, -0.28, 0.3));
  EXPECT_FALSE(clearance.permits(map, 0.0, 0.0, -M_PI_4, 0.08, 0.30));
}

TEST(TerminalParkingTest, ReportsExactClearanceRejectionReason)
{
  using nav2_costmap_2d::FREE_SPACE;
  using nav2_costmap_2d::LETHAL_OBSTACLE;
  nav2_costmap_2d::Costmap2D map(80, 80, 0.05, -2.0, -2.0, FREE_SPACE);
  TerminalClearance clearance(map);
  TerminalClearance::Report report;
  EXPECT_FALSE(clearance.permits(map, 0.0, 0.0, 0.0, 0.08, 0.30, &report));
  EXPECT_EQ(report.reason, TerminalClearance::RejectReason::Unobserved);
  EXPECT_GT(report.unknown_cells, 0u);

  const tf2::Vector3 origin(0.0, 0.0, 0.8);
  for (double angle = -1.57; angle <= 1.57; angle += 0.005) {
    clearance.observe(origin, tf2::Vector3(1.2 * std::cos(angle), 1.2 * std::sin(angle), 0.4));
  }
  EXPECT_TRUE(clearance.permits(map, 0.0, 0.0, 0.0, 0.08, 0.30, &report));
  EXPECT_EQ(report.reason, TerminalClearance::RejectReason::Cleared);

  clearance.observe(origin, tf2::Vector3(0.36, 0.0, 0.14));
  EXPECT_FALSE(clearance.permits(map, 0.0, 0.0, 0.0, 0.08, 0.30, &report));
  EXPECT_EQ(report.reason, TerminalClearance::RejectReason::CloudLethal);
  EXPECT_EQ(report.cloud_hits, 1u);
  EXPECT_NEAR(report.cloud_z, 0.14, 1e-4);

  unsigned int cx, cy;
  ASSERT_TRUE(map.worldToMap(0.32, 0.0, cx, cy));
  map.setCost(cx, cy, LETHAL_OBSTACLE);
  EXPECT_FALSE(clearance.permits(map, 0.0, 0.0, 0.0, 0.08, 0.30, &report));
  EXPECT_EQ(report.reason, TerminalClearance::RejectReason::CostmapLethal);
  EXPECT_EQ(report.costmap_cost, LETHAL_OBSTACLE);
}
}  // namespace odin_nav_sim
