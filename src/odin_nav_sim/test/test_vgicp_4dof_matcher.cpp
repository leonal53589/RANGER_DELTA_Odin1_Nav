#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "../src/vgicp_4dof_matcher.hpp"

namespace odin_nav_sim
{
namespace
{

TEST(VgicpCandidateReviewTest, PrefersAConsistentRegionalAdvantageOverOverlap)
{
  VgicpCandidateReview review;
  VgicpCandidateScore first;
  first.overlap_8cm = 0.90;
  first.balanced_rms_m = 0.035;
  first.cell_rms_m.assign(6, 0.035);
  VgicpCandidateScore second = first;
  second.base_xy_yaw.x() = 0.15;
  second.overlap_8cm = 0.97;
  second.balanced_rms_m = 0.045;
  second.cell_rms_m.assign(6, 0.045);
  VgicpCandidateScore third = second;
  third.base_xy_yaw.x() = -0.15;
  third.cell_rms_m.assign(6, 0.049);
  third.balanced_rms_m = 0.049;
  review.candidates = {second, first, third};
  rankVgicpCandidates(review);
  ASSERT_TRUE(review.winner);
  EXPECT_EQ(*review.winner, 1u);
  EXPECT_EQ(review.supporting_cells, 6u);
  EXPECT_NEAR(review.margin_without_best_cell_m, 0.010, 1e-9);
}

TEST(VgicpCandidateReviewTest, RejectsWinnerDependingOnOneCell)
{
  VgicpCandidateReview review;
  VgicpCandidateScore leader, runner_up;
  leader.cell_rms_m = {0.005, 0.027, 0.027, 0.027, 0.03, 0.03};
  runner_up.base_xy_yaw.x() = 0.15;
  runner_up.cell_rms_m = {0.080, 0.031, 0.031, 0.031, 0.02, 0.02};
  leader.balanced_rms_m = 0.0243;
  runner_up.balanced_rms_m = 0.0355;
  review.candidates = {leader, runner_up};
  rankVgicpCandidates(review);
  EXPECT_FALSE(review.winner);
  EXPECT_EQ(review.reason, "insufficient_spatial_support");
}

TEST(VgicpCandidateReviewTest, RequiresConsecutiveStationaryEvidenceAgainstAFixedAnchor)
{
  VgicpCandidateStreak streak;
  Eigen::Vector3d pose(1.0, 2.0, M_PI - 0.005);
  EXPECT_EQ(streak.update(100'000'000, &pose), 1u);
  pose.z() = -M_PI + 0.005;
  for (int i = 2; i <= 5; ++i) {
    EXPECT_EQ(streak.update(i * 100'000'000, &pose), static_cast<size_t>(i));
  }
  EXPECT_TRUE(streak.consistent());
  pose.x() += 0.012;
  EXPECT_EQ(streak.update(600'000'000, &pose), 6u);
  pose.x() += 0.012;
  EXPECT_EQ(streak.update(700'000'000, &pose), 1u);
  EXPECT_FALSE(streak.consistent());
  EXPECT_EQ(streak.update(1'100'000'000, &pose), 1u);
  EXPECT_EQ(streak.update(1'200'000'000, nullptr), 0u);
  EXPECT_EQ(streak.update(1'300'000'000, &pose), 1u);
}

TEST(VgicpRecoveryWindowTest, RetainsSeparateFitAndValidationClouds)
{
  VgicpRecoveryWindow window;
  const Eigen::Vector3d parked = Eigen::Vector3d::Zero();
  const std::vector<Eigen::Vector3f> training{{1.f, 0.f, 1.f}}, validation{{2.f, 0.f, 1.f}};
  std::vector<VgicpCandidateScore> proposals(2);
  proposals[1].base_xy_yaw.x() = 0.15;
  for (size_t i = 0; i < VgicpRecoveryWindow::size; ++i) {
    const auto & points = i < VgicpRecoveryWindow::half_size ? training : validation;
    EXPECT_EQ(window.add((i + 1) * 100'000'000, parked, parked, points,
      i == 0 ? proposals : std::vector<VgicpCandidateScore>{}),
      i + 1 == VgicpRecoveryWindow::size);
  }
  EXPECT_EQ(window.proposals().size(), 2u);
  EXPECT_EQ(window.clouds()[0].size(), 6u);
  EXPECT_EQ(window.clouds()[1].size(), 6u);
  EXPECT_EQ(window.clouds()[0].back().x(), 1.0f);
  EXPECT_EQ(window.clouds()[1].front().x(), 2.0f);
}

TEST(VgicpRecoveryWindowTest, DiscardsEvidenceAcrossMotionOrInputGap)
{
  VgicpRecoveryWindow window;
  const Eigen::Vector3d parked = Eigen::Vector3d::Zero();
  const std::vector<Eigen::Vector3f> points{{1.f, 0.f, 1.f}};
  window.add(100'000'000, parked, parked, points, {});
  window.add(200'000'000, parked, parked, points, {});
  EXPECT_EQ(window.count(), 2u);
  window.add(300'000'000, {0.016, 0., 0.}, parked, points, {});
  EXPECT_EQ(window.count(), 1u);
  window.add(700'000'000, {0.016, 0., 0.}, parked, points, {});
  EXPECT_EQ(window.count(), 1u);
  window.add(800'000'000, {0.016, 0., 0.}, parked, points, {});
  window.add(900'000'000, {0.016, 0., 0.}, {0.20, 0., 0.}, points, {});
  EXPECT_EQ(window.count(), 1u);
}

TEST(VgicpOdomBudgetTest, StopsAtAnyIndependentLimit)
{
  EXPECT_TRUE(withinVgicpOdomBudget(4.0, 0.75, 0.10, 8.0, 0.8, 0.50));
  EXPECT_FALSE(withinVgicpOdomBudget(8.1, 0.75, 0.10, 8.0, 0.8, 0.50));
  EXPECT_FALSE(withinVgicpOdomBudget(4.0, 0.81, 0.10, 8.0, 0.8, 0.50));
  EXPECT_FALSE(withinVgicpOdomBudget(4.0, 0.10, 0.51, 8.0, 0.8, 0.50));
}

TEST(StaticVoxelSearchTest, PreservesReferenceIndicesAcrossBoundariesAndTies)
{
  small_gicp::PointCloud points;
  for (int x = -2; x <= 2; ++x) {
    for (int y = -2; y <= 2; ++y) {
      points.points.emplace_back(x + 0.5, y + 0.5, 0.5, 1.0);
      points.covs.push_back(Eigen::Matrix4d::Identity());
    }
  }
  auto voxels = small_gicp::create_gaussian_voxelmap(points, 1.0);
  voxels->set_search_offsets(27);
  StaticVoxelSearch search(voxels);
  for (int ix = -16; ix <= 16; ++ix) {
    for (int iy = -16; iy <= 16; ++iy) {
      const Eigen::Vector4d query(ix * 0.25, iy * 0.25, 0.5, 1.0);
      size_t expected_idx, actual_idx;
      double expected_d, actual_d;
      const auto count = voxels->nearest_neighbor_search(query, &expected_idx, &expected_d);
      EXPECT_EQ(count, search.nearest(query, &actual_idx, &actual_d));
      EXPECT_EQ(expected_idx, actual_idx);
      EXPECT_DOUBLE_EQ(expected_d, actual_d);
    }
  }
}

std::vector<Eigen::Vector3f> buildLShapedRoomMap()
{
  std::vector<Eigen::Vector3f> pts;
  auto add_wall_x = [&](double y0, double y1, double x_val) {
      for (double y = y0; y <= y1; y += 0.05) {
        for (double z = 0.15; z <= 2.20; z += 0.06) {
          pts.emplace_back(static_cast<float>(x_val), static_cast<float>(y), static_cast<float>(z));
        }
      }
    };
  auto add_wall_y = [&](double x0, double x1, double y_val) {
      for (double x = x0; x <= x1; x += 0.05) {
        for (double z = 0.15; z <= 2.20; z += 0.06) {
          pts.emplace_back(static_cast<float>(x), static_cast<float>(y_val), static_cast<float>(z));
        }
      }
    };

  add_wall_x(-4.0, 4.0, 3.5);
  add_wall_x(-4.0, 4.0, -2.5);
  add_wall_y(-2.5, 3.5, 4.0);
  add_wall_y(-2.5, 3.5, -4.0);
  // Add an interior pillar so all yaw seeds have crisp corners.
  add_wall_x(0.8, 1.5, 1.2);
  add_wall_y(1.2, 1.9, 0.8);
  return pts;
}

std::vector<Eigen::Vector3f> transformScanToRadarLocal(
  const std::vector<Eigen::Vector3f> & map_pts, double rx, double ry, double ryaw)
{
  const double c = std::cos(-ryaw);
  const double s = std::sin(-ryaw);
  std::vector<Eigen::Vector3f> local;
  local.reserve(map_pts.size());
  for (const auto & p : map_pts) {
    const double dx = p.x() - rx;
    const double dy = p.y() - ry;
    const double lx = c * dx - s * dy;
    const double ly = s * dx + c * dy;
    if (lx > 0.35 && std::hypot(lx, ly) < 7.5) {
      local.emplace_back(static_cast<float>(lx), static_cast<float>(ly), p.z());
    }
  }
  return local;
}

TEST(Vgicp4DofMatcherTest, ConvergesWithinMillimetersFromPerturbedSeed)
{
  const auto map_pts = buildLShapedRoomMap();
  MountExtrinsics mount;
  MatcherConfig cfg;
  cfg.num_threads = 1;
  Vgicp4DofMatcher matcher(map_pts, mount, cfg);

  const double true_rx = 0.42;
  const double true_ry = -0.35;
  const double true_yaw = 0.38;
  const auto scan = transformScanToRadarLocal(map_pts, true_rx, true_ry, true_yaw);
  ASSERT_GT(scan.size(), 500u);

  const Eigen::Vector3d seed(true_rx + 0.16, true_ry - 0.14, true_yaw - 0.11);
  const MatchResult res = matcher.alignGroundScan(scan, seed);
  EXPECT_TRUE(res.accepted) << res.reason;
  EXPECT_NEAR(res.radar_xy_yaw.x(), true_rx, 0.015);
  EXPECT_NEAR(res.radar_xy_yaw.y(), true_ry, 0.015);
  EXPECT_NEAR(std::remainder(res.radar_xy_yaw.z() - true_yaw, 2.0 * M_PI), 0.0, 0.015);
  EXPECT_LT(res.seed_spread_m, 0.025);
  EXPECT_GT(res.overlap_8cm, 0.90);
}

TEST(Vgicp4DofMatcherTest, StabilizesWeakLongitudinalAxisWithPriorDamping)
{
  // Parallel featureless corridor along X: Y & Yaw constrained, X completely degenerate.
  std::vector<Eigen::Vector3f> corridor;
  for (double x = -8.0; x <= 8.0; x += 0.05) {
    for (double z = 0.15; z <= 2.20; z += 0.06) {
      corridor.emplace_back(static_cast<float>(x), -1.5f, static_cast<float>(z));
      corridor.emplace_back(static_cast<float>(x), 1.5f, static_cast<float>(z));
    }
  }
  MountExtrinsics mount;
  MatcherConfig cfg;
  cfg.num_threads = 1;
  Vgicp4DofMatcher matcher(corridor, mount, cfg);

  const auto scan = transformScanToRadarLocal(corridor, 0.0, 0.0, 0.0);
  const Eigen::Vector3d seed(0.0, 0.10, 0.07);
  const MatchResult res = matcher.alignGroundScan(scan, seed);
  // Prior damping holds the unobservable axis near seed while constraining Y,
  // and validation safely rejects the completely featureless corridor.
  EXPECT_FALSE(res.accepted);
  EXPECT_LT(res.geometry_singular_ratio, 0.03);
  EXPECT_NEAR(res.radar_xy_yaw.y(), 0.0, 0.02);
  EXPECT_LT(std::abs(res.radar_xy_yaw.x()), 0.08);
}

TEST(Vgicp4DofMatcherTest, DeskewsMovingScanOverOffsetTime)
{
  const auto map_pts = buildLShapedRoomMap();
  MountExtrinsics mount;
  MatcherConfig cfg;
  cfg.num_threads = 1;
  Vgicp4DofMatcher matcher(map_pts, mount, cfg);

  // True base at scan start (0.20, -0.20, 0.25), spinning +0.30 rad/s and driving +0.30 m/s.
  const Eigen::Vector3d base_start(0.20, -0.20, 0.25);
  const double vx = 0.30;
  const double wz = 0.30;

  std::vector<RawLidarPoint> raw_scan;
  std::vector<MotionDeltaSE2> track;
  for (int k = 0; k <= 10; ++k) {
    const double dt = 0.10 * (static_cast<double>(k) / 10.0);
    const double dyaw = wz * dt;
    const double arc = (wz != 0.0) ? (vx / wz) * std::sin(dyaw) : vx * dt;
    const double dx = vx * dt;
    track.push_back({dt, dx, 0.5 * vx * dyaw * dt, dyaw});
    (void)arc;
  }

  const Eigen::Matrix3d r_bl = mount.fullRotation();
  const Eigen::Vector3d t_bl(mount.tx, mount.ty, mount.tz);
  for (size_t i = 0; i < map_pts.size(); ++i) {
    const double dt = 0.10 * static_cast<double>(i % 10) / 10.0;
    const MotionDeltaSE2 d = detail::interpolateMotion(track, dt);
    const double cb = std::cos(base_start.z());
    const double sb = std::sin(base_start.z());
    const double bx = base_start.x() + cb * d.x - sb * d.y;
    const double by = base_start.y() + sb * d.x + cb * d.y;
    const double byaw = base_start.z() + d.yaw;

    const double ci = std::cos(-byaw);
    const double si = std::sin(-byaw);
    const double pdx = map_pts[i].x() - bx;
    const double pdy = map_pts[i].y() - by;
    const Eigen::Vector3d p_base(ci * pdx - si * pdy, si * pdx + ci * pdy, map_pts[i].z());
    const Eigen::Vector3d p_lidar = r_bl.transpose() * (p_base - t_bl);
    if (p_lidar.x() > 0.35 && p_lidar.norm() < 7.5) {
      RawLidarPoint rp;
      rp.x = static_cast<float>(p_lidar.x());
      rp.y = static_cast<float>(p_lidar.y());
      rp.z = static_cast<float>(p_lidar.z());
      rp.offset_s = static_cast<float>(dt);
      raw_scan.push_back(rp);
    }
  }

  const auto deskewed = deskewAndGroundFilter(
    raw_scan, mount, track, cfg.ground_voxel, cfg.height_min, cfg.height_max);
  const Eigen::Vector3d radar_start = mount.baseToRadar(base_start);
  const Eigen::Vector3d seed = radar_start + Eigen::Vector3d(0.08, -0.06, 0.04);
  const MatchResult res = matcher.alignGroundScan(deskewed, seed);
  EXPECT_TRUE(res.accepted) << res.reason;
  EXPECT_NEAR(res.base_xy_yaw.x(), base_start.x(), 0.02);
  EXPECT_NEAR(res.base_xy_yaw.y(), base_start.y(), 0.02);
  EXPECT_NEAR(std::remainder(res.base_xy_yaw.z() - base_start.z(), 2.0 * M_PI), 0.0, 0.02);
}

TEST(Vgicp4DofMatcherTest, RejectsScanWithLowOverlap)
{
  const auto map_pts = buildLShapedRoomMap();
  MatcherConfig cfg;
  cfg.num_threads = 1;
  Vgicp4DofMatcher matcher(map_pts, MountExtrinsics{}, cfg);

  const auto scan = transformScanToRadarLocal(map_pts, 0.0, 0.0, 0.0);
  // Seed shifted far away (2.0 m) so overlap fails.
  const MatchResult res = matcher.alignGroundScan(scan, {2.0, 2.0, 0.0}, 5.0, 1.0);
  EXPECT_FALSE(res.accepted);
}

}  // namespace
}  // namespace odin_nav_sim
