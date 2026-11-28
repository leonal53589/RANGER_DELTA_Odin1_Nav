#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <array>
#include <utility>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <Eigen/Core>

namespace odin_nav_sim
{

struct VgicpCandidateScore
{
  Eigen::Vector3d base_xy_yaw{Eigen::Vector3d::Zero()};
  double z_m{0.0};
  double overlap_8cm{0.0};
  double balanced_rms_m{0.0};
  std::vector<double> cell_rms_m;
};

struct VgicpCandidateReview
{
  std::string reason{"not_requested"};
  std::vector<VgicpCandidateScore> candidates;
  std::optional<size_t> winner;
  size_t supporting_cells{0};
  double margin_without_best_cell_m{0.0};
  double local_minimum_margin_m{0.0};
};

inline bool isVgicpRecoveryInput(std::string_view reason)
{
  return reason == "ambiguous_fit" || reason == "insufficient_converged_fits";
}

inline bool sameVgicpCandidate(const VgicpCandidateScore & a, const VgicpCandidateScore & b)
{
  const Eigen::Vector3d delta = a.base_xy_yaw - b.base_xy_yaw;
  return delta.head<2>().norm() <= 0.03 &&
         std::abs(std::remainder(delta.z(), 2.0 * M_PI)) <= 2.0 * M_PI / 180.0;
}

// Each candidate must be scored on the same validation points, grouped into the
// same cells. Compare the leader against every geometrically distinct rival.
inline void rankVgicpCandidates(VgicpCandidateReview & review)
{
  if (review.candidates.size() < 2) {
    review.reason = "insufficient_distinct_candidates";
    return;
  }
  if (review.candidates.front().cell_rms_m.size() < 6) {
    review.reason = "insufficient_validation_cells";
    return;
  }
  auto best = std::min_element(
    review.candidates.begin(), review.candidates.end(), [](const auto & a, const auto & b) {
      return a.balanced_rms_m < b.balanced_rms_m;
    });
  review.supporting_cells = best->cell_rms_m.size();
  review.margin_without_best_cell_m = std::numeric_limits<double>::infinity();
  bool has_rival = false;
  for (const auto & rival : review.candidates) {
    if (sameVgicpCandidate(rival, *best)) {
      continue;
    }
    has_rival = true;
    size_t winning_cells = 0;
    double total = 0.0;
    double largest = 0.0;
    for (size_t i = 0; i < best->cell_rms_m.size(); ++i) {
      const double difference = rival.cell_rms_m[i] - best->cell_rms_m[i];
      total += difference;
      largest = std::max(largest, difference);
      winning_cells += difference >= 0.003;
    }
    const double margin = (total - largest) / (best->cell_rms_m.size() - 1);
    review.supporting_cells = std::min(review.supporting_cells, winning_cells);
    review.margin_without_best_cell_m = std::min(review.margin_without_best_cell_m, margin);
  }
  if (!has_rival) {
    review.supporting_cells = 0;
    review.margin_without_best_cell_m = 0.0;
    review.reason = "insufficient_distinct_candidates";
    return;
  }
  if (review.supporting_cells < 3 || review.margin_without_best_cell_m < 0.002) {
    review.reason = "insufficient_spatial_support";
    return;
  }
  review.winner = std::distance(review.candidates.begin(), best);
  review.reason = "candidate_supported";
}

// Reject a window when the parked pose, input clock, or seed jumps.
class VgicpRecoveryWindow
{
public:
  static constexpr size_t half_size = 6;
  static constexpr size_t size = 2 * half_size;

  bool add(
    int64_t stamp, const Eigen::Vector3d & wheel_pose, const Eigen::Vector3d & seed,
    const std::vector<Eigen::Vector3f> & points,
    const std::vector<VgicpCandidateScore> & proposals)
  {
    if (count_ == size || (count_ && (stamp <= last_stamp_ || stamp - last_stamp_ > 250'000'000 ||
      !near(wheel_pose, anchor_, 0.008, 0.01) || !near(seed, seed_, 0.10, 0.09))))
    {
      reset();
    }
    if (count_ == 0) {
      anchor_ = wheel_pose;
      seed_ = seed;
    }
    last_stamp_ = stamp;
    const size_t half = count_ / half_size;
    clouds_[half].insert(clouds_[half].end(), points.begin(), points.end());
    if (half == 0) {
      for (const auto & proposal : proposals) {
        if (std::none_of(proposals_.begin(), proposals_.end(), [&](const auto & previous) {
            return sameVgicpCandidate(previous, proposal);
          }))
        {
          proposals_.push_back(proposal);
        }
      }
    }
    return ++count_ == size;
  }

  void reset()
  {
    count_ = 0;
    clouds_[0].clear();
    clouds_[1].clear();
    proposals_.clear();
  }

  const auto & clouds() const {return clouds_;}
  const auto & proposals() const {return proposals_;}
  size_t count() const {return count_;}

  static bool near(
    const Eigen::Vector3d & a, const Eigen::Vector3d & b, double xy, double yaw)
  {
    const Eigen::Vector3d delta = a - b;
    return delta.head<2>().norm() <= xy &&
           std::abs(std::remainder(delta.z(), 2.0 * M_PI)) <= yaw;
  }

private:
  Eigen::Vector3d anchor_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d seed_{Eigen::Vector3d::Zero()};
  int64_t last_stamp_{0};
  size_t count_{0};
  std::array<std::vector<Eigen::Vector3f>, 2> clouds_;
  std::vector<VgicpCandidateScore> proposals_;
};

inline bool withinVgicpOdomBudget(
  double age, double distance, double turn, double limit_age, double limit_distance,
  double limit_turn)
{
  return age >= 0.0 && age <= limit_age && distance <= limit_distance && turn <= limit_turn;
}

// Compare to a fixed anchor so a sequence of small drifts cannot build a streak.
class VgicpCandidateStreak
{
public:
  size_t update(int64_t stamp_ns, const Eigen::Vector3d * pose)
  {
    if (!pose) {
      count_ = 0;
      return count_;
    }
    if (count_ == 0 || stamp_ns <= last_stamp_ || stamp_ns - last_stamp_ > 300'000'000 ||
      (*pose - anchor_).head<2>().norm() > 0.02 ||
      std::abs(std::remainder(pose->z() - anchor_.z(), 2.0 * M_PI)) > 1.5 * M_PI / 180.0)
    {
      anchor_ = *pose;
      first_stamp_ = stamp_ns;
      count_ = 0;
    }
    last_stamp_ = stamp_ns;
    ++count_;
    return count_;
  }

  bool consistent() const
  {
    return count_ >= 5 && last_stamp_ - first_stamp_ >= 300'000'000;
  }

private:
  Eigen::Vector3d anchor_{Eigen::Vector3d::Zero()};
  int64_t first_stamp_{0};
  int64_t last_stamp_{0};
  size_t count_{0};
};

}  // namespace odin_nav_sim
