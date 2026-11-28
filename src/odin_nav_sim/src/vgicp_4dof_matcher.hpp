#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <small_gicp/ann/gaussian_voxelmap.hpp>
#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration_helper.hpp>
#include <small_gicp/registration/rejector.hpp>

#include "static_voxel_search.hpp"
#include "vgicp_candidate_review.hpp"

namespace odin_nav_sim
{

struct MountExtrinsics
{
  double tx{0.1754};
  double ty{0.0025};
  double tz{1.1241};
  double roll{-0.00085};
  double pitch{0.01325};
  double yaw{-0.02631};

  Eigen::Matrix3d levelRotation() const
  {
    return (Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
  }

  Eigen::Matrix3d fullRotation() const
  {
    return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
  }

  Eigen::Vector3d radarToBase(const Eigen::Vector3d & radar_xy_yaw) const
  {
    const double base_yaw = std::remainder(radar_xy_yaw.z() - yaw, 2.0 * M_PI);
    const double c = std::cos(base_yaw);
    const double s = std::sin(base_yaw);
    return {
      radar_xy_yaw.x() - (c * tx - s * ty),
      radar_xy_yaw.y() - (s * tx + c * ty),
      base_yaw,
    };
  }

  Eigen::Vector3d baseToRadar(const Eigen::Vector3d & base_xy_yaw) const
  {
    const double c = std::cos(base_xy_yaw.z());
    const double s = std::sin(base_xy_yaw.z());
    return {
      base_xy_yaw.x() + (c * tx - s * ty),
      base_xy_yaw.y() + (s * tx + c * ty),
      std::remainder(base_xy_yaw.z() + yaw, 2.0 * M_PI),
    };
  }
};

struct RawLidarPoint
{
  float x{};
  float y{};
  float z{};
  float offset_s{};
};

struct MotionDeltaSE2
{
  double dt_s{};
  double x{};
  double y{};
  double yaw{};
};

struct MatcherConfig
{
  double target_preprocess_voxel{0.075};
  double source_preprocess_voxel{0.075};
  double ground_voxel{0.075};
  double height_min{0.10};
  double height_max{3.00};
  double coarse_voxel{0.16};
  double fine_voxel{0.10};
  double coarse_max_dist{0.35};
  double fine_max_dist{0.20};
  int coarse_max_iters{20};
  int fine_max_iters{25};
  double weak_eigen_ratio{0.08};
  double weak_damp{300.0};
  double seed_yaw_offset_deg{6.0};
  double max_seed_spread_m{0.025};
  double max_seed_spread_rad{2.0 * M_PI / 180.0};
  double min_overlap_8cm{0.65};
  double min_inlier_ratio{0.45};
  double min_geometry_ratio{0.03};
  double max_innovation_xy{0.35};
  double max_innovation_yaw{20.0 * M_PI / 180.0};
  double max_innovation_z{0.15};
  int num_threads{2};
};

struct MatchResult
{
  bool accepted{false};
  std::string reason{"uninitialized"};
  Eigen::Vector3d radar_xy_yaw{Eigen::Vector3d::Zero()};
  Eigen::Vector3d base_xy_yaw{Eigen::Vector3d::Zero()};
  double z_m{0.0};
  double overlap_8cm{0.0};
  double inlier_ratio{0.0};
  double geometry_singular_ratio{0.0};
  double seed_spread_m{0.0};
  double seed_spread_rad{0.0};
  double solve_ms{0.0};
  VgicpCandidateReview review;
};

namespace detail
{
struct VoxelKey
{
  int32_t ix, iy, iz;
  bool operator==(const VoxelKey & o) const
  {
    return ix == o.ix && iy == o.iy && iz == o.iz;
  }
};

struct VoxelKeyHash
{
  size_t operator()(const VoxelKey & k) const noexcept
  {
    size_t h = static_cast<size_t>(k.ix) * 73856093u;
    h ^= static_cast<size_t>(k.iy) * 19349663u;
    h ^= static_cast<size_t>(k.iz) * 83492791u;
    return h;
  }
};

inline std::vector<Eigen::Vector3f> voxelFirstSample(
  const std::vector<Eigen::Vector3f> & pts, double voxel)
{
  std::vector<Eigen::Vector3f> out;
  out.reserve(pts.size());
  std::unordered_map<VoxelKey, bool, VoxelKeyHash> seen;
  seen.reserve(pts.size());
  const double inv = 1.0 / voxel;
  for (const auto & p : pts) {
    const VoxelKey key{
      static_cast<int32_t>(std::floor(p.x() * inv)),
      static_cast<int32_t>(std::floor(p.y() * inv)),
      static_cast<int32_t>(std::floor(p.z() * inv)),
    };
    if (seen.emplace(key, true).second) {
      out.push_back(p);
    }
  }
  return out;
}

inline double yawOf(const Eigen::Matrix3d & r)
{
  return std::atan2(r(1, 0), r(0, 0));
}

inline std::pair<Eigen::Matrix3d, double> fitFloorOrDefault(
  const std::vector<Eigen::Vector3f> & lidar_pts, const MountExtrinsics & mount)
{
  const Eigen::Matrix3d nom_level = mount.levelRotation();
  const double nom_h = mount.tz;

  std::vector<Eigen::Vector3f> floor_pts;
  floor_pts.reserve(lidar_pts.size() / 4);
  for (const auto & p : lidar_pts) {
    if (p.x() > 0.35f && p.x() < 4.0f && std::abs(p.y()) < 2.2f &&
      p.z() < -0.80f && p.z() > -1.45f)
    {
      floor_pts.push_back(p);
    }
  }
  const auto grid = voxelFirstSample(floor_pts, 0.025);
  if (grid.size() < 40) {
    return {nom_level, nom_h};
  }

  // Median initial z offset + IRLS robust plane z = a*x + b*y + c.
  std::vector<double> zs;
  zs.reserve(grid.size());
  for (const auto & p : grid) {
    zs.push_back(p.z());
  }
  std::nth_element(zs.begin(), zs.begin() + zs.size() / 2, zs.end());
  Eigen::Vector3d coeff(0.0, 0.0, zs[zs.size() / 2]);

  constexpr double f_scale = 0.005;
  for (int it = 0; it < 6; ++it) {
    Eigen::Matrix3d ata = Eigen::Matrix3d::Zero();
    Eigen::Vector3d atb = Eigen::Vector3d::Zero();
    for (const auto & p : grid) {
      const double r = coeff.x() * p.x() + coeff.y() * p.y() + coeff.z() - p.z();
      const double w = 1.0 / (1.0 + (r / f_scale) * (r / f_scale));
      const Eigen::Vector3d row(p.x(), p.y(), 1.0);
      ata += w * (row * row.transpose());
      atb += w * row * static_cast<double>(p.z());
    }
    const Eigen::Vector3d next = ata.ldlt().solve(atb);
    if (!next.allFinite()) {
      return {nom_level, nom_h};
    }
    coeff = next;
  }

  Eigen::Vector3d n(-coeff.x(), -coeff.y(), 1.0);
  const double norm = n.norm();
  if (norm < 1e-6) {
    return {nom_level, nom_h};
  }
  n /= norm;
  const double roll = std::atan2(n.y(), n.z());
  const double pitch = std::atan2(-n.x(), std::hypot(n.y(), n.z()));
  const double cand_h = -coeff.z() * n.z();
  const Eigen::Matrix3d cand_level =
    (Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();

  if (cand_h <= 1.05 || cand_h >= 1.20 ||
    (cand_level.col(2) - nom_level.col(2)).norm() >= 0.07)
  {
    return {nom_level, nom_h};
  }
  return {cand_level, cand_h};
}

inline MotionDeltaSE2 interpolateMotion(
  const std::vector<MotionDeltaSE2> & track, double dt_s)
{
  if (track.empty()) {
    return {};
  }
  if (dt_s <= track.front().dt_s) {
    return track.front();
  }
  if (dt_s >= track.back().dt_s) {
    return track.back();
  }
  auto it = std::upper_bound(
    track.begin(), track.end(), dt_s,
    [](double val, const MotionDeltaSE2 & s) {return val < s.dt_s;});
  const auto & b = *it;
  const auto & a = *std::prev(it);
  const double span = b.dt_s - a.dt_s;
  const double u = span > 1e-9 ? (dt_s - a.dt_s) / span : 0.0;
  const double dyaw = std::remainder(b.yaw - a.yaw, 2.0 * M_PI);
  return {
    dt_s,
    a.x + u * (b.x - a.x),
    a.y + u * (b.y - a.y),
    std::remainder(a.yaw + u * dyaw, 2.0 * M_PI),
  };
}
}  // namespace detail

/// Load binary-float32 or ASCII XYZ points from a .pcd file.
inline std::vector<Eigen::Vector3f> loadPcdXyz(const std::string & path)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("Could not open PCD file: " + path);
  }
  std::vector<std::string> fields;
  std::vector<int> sizes;
  std::vector<char> types;
  std::vector<int> counts;
  size_t num_points = 0;
  std::string data_mode;

  std::string line;
  while (std::getline(input, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream iss(line);
    std::string key;
    iss >> key;
    if (key == "FIELDS") {
      std::string f;
      while (iss >> f) {fields.push_back(f);}
    } else if (key == "SIZE") {
      int v;
      while (iss >> v) {sizes.push_back(v);}
    } else if (key == "TYPE") {
      std::string v;
      while (iss >> v) {types.push_back(v[0]);}
    } else if (key == "COUNT") {
      int v;
      while (iss >> v) {counts.push_back(v);}
    } else if (key == "POINTS") {
      iss >> num_points;
    } else if (key == "DATA") {
      iss >> data_mode;
      break;
    }
  }

  if (fields.size() < 3 || fields[0] != "x" || fields[1] != "y" || fields[2] != "z") {
    throw std::runtime_error("Unsupported PCD FIELDS layout in " + path);
  }
  if (sizes.size() != fields.size() || types.size() != fields.size()) {
    throw std::runtime_error("Incomplete PCD header in " + path);
  }

  std::vector<Eigen::Vector3f> points;
  points.reserve(num_points);

  if (data_mode == "binary") {
    size_t point_step = 0;
    for (size_t i = 0; i < fields.size(); ++i) {
      const int c = i < counts.size() ? counts[i] : 1;
      point_step += static_cast<size_t>(sizes[i]) * static_cast<size_t>(c);
    }
    if (sizes[0] != 4 || sizes[1] != 4 || sizes[2] != 4 ||
      types[0] != 'F' || types[1] != 'F' || types[2] != 'F')
    {
      throw std::runtime_error("Binary PCD requires float32 XYZ in " + path);
    }
    std::vector<uint8_t> buf(point_step);
    while (input.read(reinterpret_cast<char *>(buf.data()), static_cast<std::streamsize>(point_step))) {
      Eigen::Vector3f p;
      std::memcpy(&p.x(), buf.data(), 4);
      std::memcpy(&p.y(), buf.data() + 4, 4);
      std::memcpy(&p.z(), buf.data() + 8, 4);
      if (p.allFinite()) {
        points.push_back(p);
      }
    }
  } else if (data_mode == "ascii") {
    std::string row;
    while (std::getline(input, row)) {
      std::istringstream iss(row);
      Eigen::Vector3f p;
      if ((iss >> p.x() >> p.y() >> p.z()) && p.allFinite()) {
        points.push_back(p);
      }
    }
  } else {
    throw std::runtime_error("Unsupported PCD DATA mode (" + data_mode + ") in " + path);
  }
  return points;
}

/// Deskew raw lidar points using relative chassis SE(2) samples over [0, max_offset],
/// level to ground, filter [height_min, height_max], and downsample.
inline std::vector<Eigen::Vector3f> deskewAndGroundFilter(
  const std::vector<RawLidarPoint> & raw_points,
  const MountExtrinsics & mount,
  const std::vector<MotionDeltaSE2> & motion_track,
  double ground_voxel,
  double height_min,
  double height_max)
{
  const bool do_deskew = motion_track.size() >= 2;
  const Eigen::Matrix3d r_bl = mount.fullRotation();
  const Eigen::Matrix3d r_lb = r_bl.transpose();
  const Eigen::Vector3d t_bl(mount.tx, mount.ty, mount.tz);

  std::vector<Eigen::Vector3f> lidar_pts;
  lidar_pts.reserve(raw_points.size());
  for (const auto & pt : raw_points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z) ||
      pt.x <= 0.35f)
    {
      continue;
    }
    const double r2 = static_cast<double>(pt.x) * pt.x +
      static_cast<double>(pt.y) * pt.y + static_cast<double>(pt.z) * pt.z;
    if (r2 >= 64.0) {
      continue;
    }
    if (!do_deskew || !std::isfinite(pt.offset_s) || std::abs(pt.offset_s) < 1e-5f) {
      lidar_pts.emplace_back(pt.x, pt.y, pt.z);
    } else {
      // Transform lidar(t_p) -> base(t_p) -> base(t_scan) -> lidar(t_scan).
      const MotionDeltaSE2 d = detail::interpolateMotion(motion_track, pt.offset_s);
      const Eigen::Vector3d p_b = r_bl * Eigen::Vector3d(pt.x, pt.y, pt.z) + t_bl;
      const double c = std::cos(d.yaw);
      const double s = std::sin(d.yaw);
      const Eigen::Vector3d p_b0(c * p_b.x() - s * p_b.y() + d.x,
        s * p_b.x() + c * p_b.y() + d.y, p_b.z());
      const Eigen::Vector3d p_l0 = r_lb * (p_b0 - t_bl);
      lidar_pts.emplace_back(
        static_cast<float>(p_l0.x()),
        static_cast<float>(p_l0.y()),
        static_cast<float>(p_l0.z()));
    }
  }

  const auto [level, height] = detail::fitFloorOrDefault(lidar_pts, mount);
  const Eigen::Matrix3f lf = level.cast<float>();
  const float hf = static_cast<float>(height);

  std::vector<Eigen::Vector3f> ground;
  ground.reserve(lidar_pts.size());
  for (const auto & p : lidar_pts) {
    Eigen::Vector3f g = lf * p;
    g.z() += hf;
    if (g.z() > static_cast<float>(height_min) && g.z() < static_cast<float>(height_max)) {
      ground.push_back(g);
    }
  }
  return detail::voxelFirstSample(ground, ground_voxel);
}

/// Resident dual-voxelmap 4-DoF VGICP matcher with coarse-to-fine search,
/// weak-Hessian eigenmode prior damping, and multi-seed convergence checks.
class Vgicp4DofMatcher
{
public:
  Vgicp4DofMatcher(
    const std::vector<Eigen::Vector3f> & map_points,
    const MountExtrinsics & mount = {},
    const MatcherConfig & config = {})
  : mount_(mount), config_(config)
  {
    if (map_points.size() < 60) {
      throw std::runtime_error("Reference map must contain at least 60 points");
    }
    auto [target, tree] = small_gicp::preprocess_points(
      map_points, config_.target_preprocess_voxel, 16, config_.num_threads);
    if (!target || target->size() < 60) {
      throw std::runtime_error("Preprocessed reference map has fewer than 60 points");
    }
    target_ = std::move(target);
    target_tree_ = std::move(tree);

    coarse_voxels_ = small_gicp::create_gaussian_voxelmap(*target_, config_.coarse_voxel);
    coarse_voxels_->set_search_offsets(27);
    fine_voxels_ = small_gicp::create_gaussian_voxelmap(*target_, config_.fine_voxel);
    fine_voxels_->set_search_offsets(27);
    coarse_search_ = std::make_shared<StaticVoxelSearch>(coarse_voxels_);
    fine_search_ = std::make_shared<StaticVoxelSearch>(fine_voxels_);

    // Precompute ParkingKeyframe-equivalent planar flags & normals for geometry checks.
    const size_t n = target_->size();
    normals_.resize(n, Eigen::Vector3d::Zero());
    planar_.assign(n, 0);
    std::array<size_t, 16> idx{};
    std::array<double, 16> sq_dist{};
    for (size_t i = 0; i < n; ++i) {
      const size_t found = target_tree_->knn_search(
        target_->point(i), 16, idx.data(), sq_dist.data());
      if (found < 6) {
        continue;
      }
      Eigen::Vector3d mean = Eigen::Vector3d::Zero();
      for (size_t k = 0; k < found; ++k) {
        mean += target_->point(idx[k]).head<3>();
      }
      mean /= static_cast<double>(found);
      Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
      for (size_t k = 0; k < found; ++k) {
        const Eigen::Vector3d d = target_->point(idx[k]).head<3>() - mean;
        cov += d * d.transpose();
      }
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov);
      if (eig.info() != Eigen::Success) {
        continue;
      }
      const Eigen::Vector3d normal = eig.eigenvectors().col(0);
      normals_[i] = normal;
      if (eig.eigenvalues()(0) < 0.20 * eig.eigenvalues()(1) &&
        std::abs(normal.z()) < 0.80)
      {
        planar_[i] = 1;
      }
    }
  }

  size_t targetSize() const {return target_->size();}
  const MountExtrinsics & mount() const {return mount_;}
  const MatcherConfig & config() const {return config_;}

  MatchResult alignGroundScan(
    const std::vector<Eigen::Vector3f> & scan_ground_pts,
    const Eigen::Vector3d & radar_seed_xy_yaw,
    double innovation_xy_override = -1.0,
    double innovation_yaw_override = -1.0,
    double yaw_offset_deg_override = -1.0,
    bool review_ambiguity = false) const
  {
    const auto t0 = std::chrono::steady_clock::now();
    MatchResult res;
    if (scan_ground_pts.size() < 60) {
      res.reason = "insufficient_points";
      return res;
    }

    const auto source = small_gicp::preprocess_points(
      scan_ground_pts, config_.source_preprocess_voxel, 16, config_.num_threads).first;
    if (!source || source->size() < 60) {
      res.reason = "insufficient_points";
      return res;
    }

    Eigen::Isometry3d nominal(Eigen::AngleAxisd(radar_seed_xy_yaw.z(), Eigen::Vector3d::UnitZ()));
    nominal.translation() = Eigen::Vector3d(radar_seed_xy_yaw.x(), radar_seed_xy_yaw.y(), 0.0);

    struct StageFit
    {
      Eigen::Isometry3d pose;
      size_t inliers{0};
      bool converged{false};
    };

    const double yaw_deg = yaw_offset_deg_override > 0.0
      ? yaw_offset_deg_override : config_.seed_yaw_offset_deg;
    const double yaw_step = yaw_deg * M_PI / 180.0;
    const std::array<double, 3> offsets{-yaw_step, 0.0, yaw_step};
    std::array<StageFit, 3> fits;

    auto run_stage = [this](
      const small_gicp::PointCloud & source, const small_gicp::GaussianVoxelMap & voxels, const StaticVoxelSearch & search,
      const Eigen::Isometry3d & seed,
      const Eigen::Isometry3d & prior,
      double max_dist,
      int max_iters) -> StageFit
    {
      StageFit fit{seed, 0, false};
      Eigen::Matrix<double, 6, 4> basis = Eigen::Matrix<double, 6, 4>::Zero();
      basis.block<3, 3>(3, 0).setIdentity();
      basis(2, 3) = 0.5;

      small_gicp::ParallelReductionOMP reduction;
      reduction.num_threads = config_.num_threads;
      small_gicp::DistanceRejector rejector;
      rejector.max_dist_sq = max_dist * max_dist;
      std::vector<small_gicp::GICPFactor> factors(source.size());
      const double prior_yaw = detail::yawOf(prior.linear());

      for (int iter = 0; iter < max_iters; ++iter) {
        auto [hessian, gradient, cost] = reduction.linearize(
          voxels, source, search, rejector, fit.pose, factors);
        fit.inliers = static_cast<size_t>(std::count_if(
          factors.begin(), factors.end(), [](const auto & f) {return f.inlier();}));
        if (fit.inliers < 60 || !hessian.allFinite() || !gradient.allFinite()) {
          fit.converged = false;
          break;
        }
        const Eigen::Matrix4d h_raw = basis.transpose() * hessian * basis;
        const Eigen::Matrix4d h = 0.5 * (h_raw + h_raw.transpose());
        const Eigen::Vector4d g = basis.transpose() * gradient;

        Eigen::Matrix4d prior_h = Eigen::Matrix4d::Zero();
        Eigen::Vector4d delta_to_prior = Eigen::Vector4d::Zero();
        if (config_.weak_damp > 0.0 && config_.weak_eigen_ratio > 0.0) {
          Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eig(h);
          if (eig.info() == Eigen::Success) {
            const double max_eig = std::max(1e-12, eig.eigenvalues()(3));
            const auto & vecs = eig.eigenvectors();
            for (int k = 0; k < 4; ++k) {
              if (eig.eigenvalues()(k) < config_.weak_eigen_ratio * max_eig) {
                prior_h += config_.weak_damp * (vecs.col(k) * vecs.col(k).transpose());
              }
            }
            delta_to_prior.head<3>() =
              fit.pose.linear().transpose() * (prior.translation() - fit.pose.translation());
            delta_to_prior[3] = 2.0 * std::remainder(
              prior_yaw - detail::yawOf(fit.pose.linear()), 2.0 * M_PI);
          }
        }

        const double prior_cost_curr = 0.5 * delta_to_prior.dot(prior_h * delta_to_prior);
        const Eigen::Matrix4d h_reg = h + prior_h;
        const Eigen::Vector4d g_reg = g - prior_h * delta_to_prior;

        double damping = std::max(1e-8, h_reg.trace() * 1e-7);
        bool improved = false;
        for (int attempt = 0; attempt < 7; ++attempt) {
          const Eigen::Vector4d step =
            (h_reg + damping * Eigen::Matrix4d::Identity()).ldlt().solve(-g_reg);
          if (!step.allFinite()) {
            break;
          }
          const Eigen::Isometry3d next = fit.pose * small_gicp::se3_exp(basis * step);
          const double next_prior_cost =
            0.5 * (delta_to_prior - step).dot(prior_h * (delta_to_prior - step));
          if (reduction.error(voxels, source, next, factors) + next_prior_cost <=
            cost + prior_cost_curr + 1e-10)
          {
            fit.pose = next;
            improved = true;
            fit.converged = step.head<3>().norm() < 0.001 && std::abs(step[3]) < 0.001;
            break;
          }
          damping *= 10.0;
        }
        if (!improved || fit.converged) {
          break;
        }
      }
      return fit;
    };

    const auto fit_from_seed = [&](const small_gicp::PointCloud & cloud, const Eigen::Isometry3d & seed) {
        const StageFit coarse = run_stage(
          cloud, *coarse_voxels_, *coarse_search_, seed, nominal,
          config_.coarse_max_dist, config_.coarse_max_iters);
        return run_stage(
          cloud, *fine_voxels_, *fine_search_, coarse.pose, nominal,
          config_.fine_max_dist, config_.fine_max_iters);
      };
    std::array<Eigen::Isometry3d, 3> seeds;
    for (size_t i = 0; i < fits.size(); ++i) {
      seeds[i] = nominal;
      seeds[i].linear() = Eigen::AngleAxisd(
        radar_seed_xy_yaw.z() + offsets[i], Eigen::Vector3d::UnitZ()).toRotationMatrix();
      fits[i] = fit_from_seed(*source, seeds[i]);
    }

    const auto & center = fits[1];
    const Eigen::Vector3d pos = center.pose.translation();
    const double fit_yaw = detail::yawOf(center.pose.linear());
    res.radar_xy_yaw = {pos.x(), pos.y(), fit_yaw};
    res.base_xy_yaw = mount_.radarToBase(res.radar_xy_yaw);
    res.z_m = pos.z();

    Eigen::Vector2d spread_axis = Eigen::Vector2d::Zero();
    double max_spread = 0.0;
    double min_yaw_d = 0.0;
    double max_yaw_d = 0.0;
    bool all_converged = true;
    double min_inlier_ratio = 1.0;
    for (size_t i = 0; i < 3; ++i) {
      all_converged = all_converged && fits[i].converged;
      min_inlier_ratio = std::min(
        min_inlier_ratio,
        static_cast<double>(fits[i].inliers) / static_cast<double>(source->size()));
      const double dy = std::remainder(detail::yawOf(fits[i].pose.linear()) - fit_yaw, 2.0 * M_PI);
      min_yaw_d = std::min(min_yaw_d, dy);
      max_yaw_d = std::max(max_yaw_d, dy);
      for (size_t j = i + 1; j < 3; ++j) {
        const Eigen::Vector2d d =
          fits[i].pose.translation().head<2>() - fits[j].pose.translation().head<2>();
        if (d.norm() > max_spread) {
          max_spread = d.norm();
          spread_axis = d;
        }
      }
    }
    res.seed_spread_m = max_spread;
    res.seed_spread_rad = max_yaw_d - min_yaw_d;
    res.inlier_ratio = min_inlier_ratio;

    measureGeometry(scan_ground_pts, center.pose, res.overlap_8cm, res.geometry_singular_ratio);

    const double max_xy = innovation_xy_override > 0.0
      ? innovation_xy_override : config_.max_innovation_xy;
    const double max_yaw = innovation_yaw_override > 0.0
      ? innovation_yaw_override : config_.max_innovation_yaw;
    const double dx = pos.x() - radar_seed_xy_yaw.x();
    const double dy = pos.y() - radar_seed_xy_yaw.y();
    const double dyaw = std::abs(std::remainder(fit_yaw - radar_seed_xy_yaw.z(), 2.0 * M_PI));

    if (!all_converged) {
      res.reason = "insufficient_converged_fits";
    } else if (res.seed_spread_m > config_.max_seed_spread_m ||
      res.seed_spread_rad > config_.max_seed_spread_rad)
    {
      res.reason = "ambiguous_fit";
    } else if (res.overlap_8cm < config_.min_overlap_8cm ||
      res.inlier_ratio < config_.min_inlier_ratio)
    {
      res.reason = "low_overlap";
    } else if (res.geometry_singular_ratio < config_.min_geometry_ratio) {
      res.reason = "weak_geometry";
    } else if (std::hypot(dx, dy) > max_xy || dyaw > max_yaw ||
      std::abs(res.z_m) > config_.max_innovation_z)
    {
      res.reason = "large_innovation";
    } else {
      res.accepted = true;
      res.reason = "ok";
    }
    if (review_ambiguity && res.reason == "ambiguous_fit") {
      res.review = reviewCandidates(
        scan_ground_pts, nominal, max_xy, max_yaw, spread_axis, fits);
    }
    res.solve_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
    return res;
  }

  MatchResult recoverStationary(
    const VgicpRecoveryWindow & window, const Eigen::Vector3d & radar_seed,
    double max_xy, double max_yaw, float min_height) const
  {
    MatchResult result;
    result.reason = "recovery_insufficient_structure";
    const auto training = recoveryCells(window.clouds()[0], min_height);
    const auto validation = recoveryCells(window.clouds()[1], min_height);
    if (training.size() < 6 || validation.size() < 6 || window.proposals().size() < 2) {
      return result;
    }

    const auto admissible = [&](const Eigen::Isometry3d & pose) {
        const Eigen::Vector3d candidate(
          pose.translation().x(), pose.translation().y(), detail::yawOf(pose.linear()));
        return VgicpRecoveryWindow::near(candidate, radar_seed, max_xy, max_yaw) &&
               std::abs(pose.translation().z()) <= config_.max_innovation_z;
      };
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> proposals;
    double training_error = std::numeric_limits<double>::infinity();
    Eigen::Isometry3d estimate = Eigen::Isometry3d::Identity();
    for (const auto & candidate : window.proposals()) {
      const auto radar = mount_.baseToRadar(candidate.base_xy_yaw);
      Eigen::Isometry3d pose(Eigen::AngleAxisd(radar.z(), Eigen::Vector3d::UnitZ()));
      pose.translation() = Eigen::Vector3d(radar.x(), radar.y(), candidate.z_m);
      if (!admissible(pose)) {
        continue;
      }
      proposals.push_back(pose);
      const double error = scoreOnValidation(pose, training).balanced_rms_m;
      if (error < training_error) {
        estimate = pose;
        training_error = error;
      }
    }

    Eigen::Vector2d axis = Eigen::Vector2d::Zero();
    for (const auto & pose : proposals) {
      const Eigen::Vector2d delta = (pose.translation() - estimate.translation()).head<2>();
      if (delta.squaredNorm() > axis.squaredNorm()) {
        axis = delta;
      }
    }
    if (axis.norm() <= config_.max_seed_spread_m) {
      result.reason = "recovery_insufficient_distinct_candidates";
      return result;
    }
    axis.normalize();

    const auto improve = [&](const Eigen::Isometry3d & pose) {
        if (!admissible(pose)) {
          return;
        }
        const double error = scoreOnValidation(pose, training).balanced_rms_m;
        if (error < training_error) {
          estimate = pose;
          training_error = error;
        }
      };
    // Refine the weak translation with the first six scans, then account
    // for small ground-level and yaw differences before scoring held-out scans.
    auto center = estimate;
    for (int step = -10; step <= 10; ++step) {
      auto probe = center;
      probe.translation().head<2>() += (0.01 * step) * axis;
      improve(probe);
    }
    center = estimate;
    for (double dz : {-0.02, 0.0, 0.02}) {
      for (double yaw_delta : {-0.25, 0.0, 0.25}) {
        auto probe = center;
        probe.translation().z() += dz;
        probe.linear() = Eigen::AngleAxisd(
          yaw_delta * M_PI / 180.0, Eigen::Vector3d::UnitZ()) * center.linear();
        improve(probe);
      }
    }
    center = estimate;
    for (int step = -2; step <= 2; ++step) {
      auto probe = center;
      probe.translation().head<2>() += (0.005 * step) * axis;
      improve(probe);
    }

    const auto score_pose = [&](const Eigen::Isometry3d & pose) {
        auto score = scoreOnValidation(pose, validation);
        score.base_xy_yaw = mount_.radarToBase({
          pose.translation().x(), pose.translation().y(), detail::yawOf(pose.linear())});
        score.z_m = pose.translation().z();
        return score;
      };
    // The first candidate was selected without consulting these six scans.
    result.review.candidates.push_back(score_pose(estimate));
    for (const auto & pose : proposals) {
      result.review.candidates.push_back(score_pose(pose));
    }
    rankVgicpCandidates(result.review);
    // Formal recovery needs broader support than the single-scan diagnostic.
    if (result.review.supporting_cells < 6 || !result.review.winner || !sameVgicpCandidate(
        result.review.candidates[0], result.review.candidates[*result.review.winner]))
    {
      result.review.winner.reset();
      result.reason = "recovery_spatial_disagreement";
      return result;
    }

    const auto & chosen = result.review.candidates[0];
    double local_margin = std::numeric_limits<double>::infinity();
    for (const double sign : {-1.0, 1.0}) {
      auto shifted = estimate;
      shifted.translation().head<2>() += (sign * 0.03) * axis;
      auto rotated = estimate;
      rotated.linear() = Eigen::AngleAxisd(
        sign * config_.max_seed_spread_rad, Eigen::Vector3d::UnitZ()) * estimate.linear();
      local_margin = std::min({
        local_margin,
        scoreOnValidation(shifted, validation).balanced_rms_m - chosen.balanced_rms_m,
        scoreOnValidation(rotated, validation).balanced_rms_m - chosen.balanced_rms_m});
    }
    result.review.local_minimum_margin_m = local_margin;
    if (local_margin < 0.0005) {
      result.review.winner.reset();
      result.reason = "recovery_weak_local_minimum";
      return result;
    }

    std::vector<Eigen::Vector3f> structure;
    for (const auto & cell : validation) {
      structure.insert(structure.end(), cell.begin(), cell.end());
    }
    measureGeometry(structure, estimate, result.overlap_8cm, result.geometry_singular_ratio);
    if (result.overlap_8cm < config_.min_overlap_8cm ||
      result.geometry_singular_ratio < config_.min_geometry_ratio)
    {
      result.review.winner.reset();
      result.reason = "recovery_geometry_rejected";
      return result;
    }
    result.radar_xy_yaw = {
      estimate.translation().x(), estimate.translation().y(), detail::yawOf(estimate.linear())};
    result.base_xy_yaw = chosen.base_xy_yaw;
    result.z_m = chosen.z_m;
    result.review.winner = 0;
    result.review.reason = "joint_recovery_supported";
    result.accepted = true;
    result.reason = "stationary_recovery";
    return result;
  }

private:
  template<class Fit>
  VgicpCandidateReview reviewCandidates(
    const std::vector<Eigen::Vector3f> & points, const Eigen::Isometry3d & nominal,
    double max_xy, double max_yaw, const Eigen::Vector2d & spread_axis,
    const std::array<Fit, 3> & fits) const
  {
    VgicpCandidateReview review;
    // The VGICP fits supply proposals. The optional one-dimensional search
    // uses the even fine voxels, with its score checked on the odd voxels.
    using Cells = std::unordered_map<
      detail::VoxelKey, std::vector<Eigen::Vector3f>, detail::VoxelKeyHash>;
    std::array<Cells, 2> subsets;
    for (const auto & point : points) {
      const auto fine = (point.array() / 0.24f).floor().cast<int>().eval();
      const auto coarse = point.head<2>().array().floor().cast<int>().eval();
      subsets[std::abs(fine.sum() % 2)][{coarse.x(), coarse.y(), 0}].push_back(point);
    }
    const auto extract_cells = [](Cells & cells) {
        std::vector<std::vector<Eigen::Vector3f>> result;
        for (auto & entry : cells) {
          if (entry.second.size() >= 12) {
            result.push_back(std::move(entry.second));
          }
        }
        return result;
      };
    const auto training = extract_cells(subsets[0]);
    const auto validation = extract_cells(subsets[1]);
    if (training.size() < 6 || validation.size() < 6) {
      review.reason = "insufficient_validation_cells";
      return review;
    }
    const double seed_yaw = detail::yawOf(nominal.linear());
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> hypotheses;
    const auto admissible = [&](const Eigen::Isometry3d & pose) {
        return (pose.translation() - nominal.translation()).head<2>().norm() <= max_xy &&
               std::abs(std::remainder(detail::yawOf(pose.linear()) - seed_yaw, 2.0 * M_PI)) <= max_yaw &&
               std::abs(pose.translation().z()) <= config_.max_innovation_z;
      };
    const auto add_hypothesis = [&](const Eigen::Isometry3d & pose) {
        if (!admissible(pose)) {
          return;
        }
        double overlap = 0.0, geometry = 0.0;
        measureGeometry(points, pose, overlap, geometry);
        if (overlap < config_.min_overlap_8cm || geometry < config_.min_geometry_ratio) {
          return;
        }
        const auto base = mount_.radarToBase({
          pose.translation().x(), pose.translation().y(), detail::yawOf(pose.linear())});
        auto score = scoreOnValidation(pose, validation);
        score.base_xy_yaw = base;
        score.z_m = pose.translation().z();
        hypotheses.push_back(pose);
        review.candidates.push_back(std::move(score));
      };

    // Rerank the original hypotheses as well as the best translation found on
    // the fitting subset. A wall can give GICP a local minimum along its tangent.
    for (const auto & fit : fits) {
      if (fit.converged) {
        add_hypothesis(fit.pose);
      }
    }
    if (hypotheses.empty()) {
      review.reason = "insufficient_distinct_candidates";
      return review;
    }
    if (spread_axis.norm() > config_.max_seed_spread_m) {
      auto best_pose = hypotheses.front();
      double best_error = std::numeric_limits<double>::infinity();
      const Eigen::Vector2d step = 0.01 * spread_axis.normalized();
      for (const auto & hypothesis : hypotheses) {
        for (int i = -15; i <= 15; ++i) {
          auto probe = hypothesis;
          probe.translation().head<2>() += i * step;
          if (!admissible(probe)) {
            continue;
          }
          const double error = scoreOnValidation(probe, training).balanced_rms_m;
          if (error < best_error) {
            best_error = error;
            best_pose = probe;
          }
        }
      }
      add_hypothesis(best_pose);
    }
    rankVgicpCandidates(review);
    if (review.winner) {
      const size_t index = *review.winner;
      const auto & score = review.candidates[index];
      const auto & pose = hypotheses[index];
      double local_margin = std::numeric_limits<double>::infinity();
      for (size_t j = 0; j < hypotheses.size(); ++j) {
        if (sameVgicpCandidate(score, review.candidates[j])) {
          continue;
        }
        Eigen::Vector2d direction =
          (hypotheses[j].translation() - pose.translation()).head<2>();
        const bool translation_test = direction.norm() > config_.max_seed_spread_m;
        for (const double sign : {-1.0, 1.0}) {
          auto probe = pose;
          if (translation_test) {
            probe.translation().head<2>() += sign * 0.03 * direction.normalized();
          } else {
            probe.linear() = Eigen::AngleAxisd(
              sign * config_.max_seed_spread_rad, Eigen::Vector3d::UnitZ()) * pose.linear();
          }
          local_margin = std::min(local_margin,
            scoreOnValidation(probe, validation).balanced_rms_m - score.balanced_rms_m);
        }
      }
      review.local_minimum_margin_m = local_margin;
      if (score.overlap_8cm < config_.min_overlap_8cm || local_margin < 0.0005) {
        review.winner.reset();
        review.reason = score.overlap_8cm < config_.min_overlap_8cm ?
          "low_validation_overlap" : "weak_local_minimum";
      }
    }
    return review;
  }

  static std::vector<std::vector<Eigen::Vector3f>> recoveryCells(
    const std::vector<Eigen::Vector3f> & points, float min_height)
  {
    const auto unique = detail::voxelFirstSample(points, 0.04);
    std::unordered_map<detail::VoxelKey, std::vector<Eigen::Vector3f>, detail::VoxelKeyHash> cells;
    for (const auto & point : unique) {
      if (point.z() >= min_height) {
        cells[{static_cast<int32_t>(std::floor(point.x())),
               static_cast<int32_t>(std::floor(point.y())), 0}].push_back(point);
      }
    }
    std::vector<std::vector<Eigen::Vector3f>> result;
    for (auto & entry : cells) {
      if (entry.second.size() >= 12) {
        result.push_back(std::move(entry.second));
      }
    }
    return result;
  }

  void measureGeometry(
    const std::vector<Eigen::Vector3f> & points, const Eigen::Isometry3d & pose,
    double & overlap, double & geometry) const
  {
    const Eigen::Vector3d pos = pose.translation();
    const double fit_yaw = detail::yawOf(pose.linear());
    // Geometry / overlap evaluation using precomputed map normals.
    const double c = std::cos(fit_yaw);
    const double s = std::sin(fit_yaw);
    size_t within_8cm = 0;
    Eigen::Matrix3d jtj = Eigen::Matrix3d::Zero();
    for (const auto & qf : points) {
      const Eigen::Vector4d w(
        c * qf.x() - s * qf.y() + pos.x(),
        s * qf.x() + c * qf.y() + pos.y(),
        static_cast<double>(qf.z()) + pos.z(),
        1.0);
      size_t nn_idx = 0;
      double nn_sq = 0.0;
      if (target_tree_->nearest_neighbor_search(w, &nn_idx, &nn_sq) == 0) {
        continue;
      }
      if (nn_sq < 0.08 * 0.08) {
        ++within_8cm;
      }
      if (nn_sq < 0.10 * 0.10 && planar_[nn_idx]) {
        const Eigen::Vector3d & n = normals_[nn_idx];
        const double ax = w.x() - pos.x();
        const double ay = w.y() - pos.y();
        const Eigen::Vector3d row(n.x(), n.y(), (ax * n.y() - ay * n.x()) * 0.5);
        jtj += row * row.transpose();
      }
    }
    overlap = static_cast<double>(within_8cm) / static_cast<double>(points.size());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> geom_eig(jtj);
    if (geom_eig.info() == Eigen::Success && geom_eig.eigenvalues()(2) > 1e-12) {
      geometry = std::sqrt(
        std::max(0.0, geom_eig.eigenvalues()(0)) / geom_eig.eigenvalues()(2));
    }
  }

  VgicpCandidateScore scoreOnValidation(
    const Eigen::Isometry3d & pose,
    const std::vector<std::vector<Eigen::Vector3f>> & cells) const
  {
    VgicpCandidateScore score;
    size_t inside = 0, total = 0;
    for (const auto & cell : cells) {
      double squared_sum = 0.0;
      for (const auto & point : cell) {
        const Eigen::Vector4d projected = (pose * point.cast<double>()).homogeneous();
        size_t index = 0;
        double squared = 0.12 * 0.12;
        if (target_tree_->nearest_neighbor_search(projected, &index, &squared) == 0) {
          squared = 0.12 * 0.12;
        }
        inside += squared < 0.08 * 0.08;
        squared_sum += std::min(squared, 0.12 * 0.12);
      }
      const double rms = std::sqrt(squared_sum / cell.size());
      score.cell_rms_m.push_back(rms);
      score.balanced_rms_m += rms;
      total += cell.size();
    }
    score.overlap_8cm = static_cast<double>(inside) / total;
    score.balanced_rms_m /= cells.size();
    return score;
  }

  MountExtrinsics mount_;
  MatcherConfig config_;
  small_gicp::PointCloud::Ptr target_;
  std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>> target_tree_;
  small_gicp::GaussianVoxelMap::Ptr coarse_voxels_;
  small_gicp::GaussianVoxelMap::Ptr fine_voxels_;
  std::shared_ptr<const StaticVoxelSearch> coarse_search_;
  std::shared_ptr<const StaticVoxelSearch> fine_search_;
  std::vector<Eigen::Vector3d> normals_;
  std::vector<uint8_t> planar_;
};

}  // namespace odin_nav_sim
