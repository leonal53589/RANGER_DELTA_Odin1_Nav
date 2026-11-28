#ifndef ODIN_NAV_SIM__TERMINAL_CLEARANCE_HPP_
#define ODIN_NAV_SIM__TERMINAL_CLEARANCE_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "tf2/LinearMath/Vector3.h"

namespace odin_nav_sim
{

constexpr double kMaxAxialTravel = 0.20;
constexpr double kAxialStoppingMargin = 0.025;
constexpr double kAxialSpeed = 0.05;
constexpr double kCrabSpeed = 0.035;

inline std::optional<double> axialSweepDistance(
  double longitudinal, double lateral, double inner_tolerance)
{
  if (!std::isfinite(longitudinal) || !std::isfinite(lateral) ||
    std::abs(lateral) >= inner_tolerance)
  {
    return std::nullopt;
  }
  const double reach = std::sqrt(inner_tolerance * inner_tolerance - lateral * lateral);
  const double length = std::abs(longitudinal) - reach + kAxialStoppingMargin;
  if (length <= kAxialStoppingMargin || length > kMaxAxialTravel) {
    return std::nullopt;
  }
  return std::copysign(length, longitudinal);
}

// Scratch grid for a single depth scan, separate from the permissive rolling costmap.
class TerminalClearance : private nav2_costmap_2d::Costmap2D
{
public:
  enum class RejectReason
  {
    Cleared,
    NoCloud,
    StaleCloud,
    TfFailure,
    OutOfBounds,
    CostmapLethal,
    CloudLethal,
    Unobserved,
    NoNewCells
  };

  struct Report
  {
    RejectReason reason{RejectReason::Cleared};
    double block_x{0.0};
    double block_y{0.0};
    double block_dist{0.0};
    double cloud_z{0.0};
    unsigned int cloud_hits{0};
    unsigned int costmap_cost{0};
    unsigned int swept_cells{0};
    unsigned int free_cells{0};
    unsigned int unknown_cells{0};
    unsigned int prior_scans_fed{0};
    unsigned int prior_scans_ok{0};
    double cloud_age{0.0};
    std::size_t live_points{0};
  };

  explicit TerminalClearance(const nav2_costmap_2d::Costmap2D & map)
  : Costmap2D(map.getSizeInCellsX(), map.getSizeInCellsY(), map.getResolution(),
      map.getOriginX(), map.getOriginY(), nav2_costmap_2d::NO_INFORMATION),
    traced_(static_cast<size_t>(map.getSizeInCellsX()) * map.getSizeInCellsY(), false),
    hit_count_(traced_.size(), 0),
    hit_z_(traced_.size(), -1.0f)
  {}

  void markRecentlyOccupied(double x, double y, double radius)
  {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(radius) || radius <= 0.0) {
      return;
    }
    const double extent = radius + getResolution() * std::sqrt(0.5);
    unsigned int x_min, y_min, x_max, y_max;
    if (!worldToMap(x - extent, y - extent, x_min, y_min) ||
      !worldToMap(x + extent, y + extent, x_max, y_max))
    {
      return;
    }
    auto * cells = getCharMap();
    const double r2 = extent * extent;
    for (unsigned int cy = y_min; cy <= y_max; ++cy) {
      for (unsigned int cx = x_min; cx <= x_max; ++cx) {
        double wx, wy;
        mapToWorld(cx, cy, wx, wy);
        const double dx = wx - x;
        const double dy = wy - y;
        if (dx * dx + dy * dy <= r2) {
          const auto idx = getIndex(cx, cy);
          if (cells[idx] == nav2_costmap_2d::NO_INFORMATION) {
            cells[idx] = nav2_costmap_2d::FREE_SPACE;
          }
        }
      }
    }
  }

  void observe(const tf2::Vector3 & origin, const tf2::Vector3 & hit)
  {
    observeImpl(origin, hit, true);
  }

  void observeFreeOnly(const tf2::Vector3 & origin, const tf2::Vector3 & hit)
  {
    observeImpl(origin, hit, false);
  }

  bool permits(
    const nav2_costmap_2d::Costmap2D & map, double x, double y, double yaw, double travel,
    double radius, Report * report = nullptr) const
  {
    if (report) {
      *report = Report{};
    }
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    const double end_x = x + travel * cos_yaw;
    const double end_y = y + travel * sin_yaw;
    // Account for cells whose area intersects the swept disk.
    const double extent = radius + getResolution() * std::sqrt(0.5);
    unsigned int x_min, y_min, x_max, y_max;
    if (!worldToMap(std::min(x, end_x) - extent, std::min(y, end_y) - extent, x_min, y_min) ||
      !worldToMap(std::max(x, end_x) + extent, std::max(y, end_y) + extent, x_max, y_max))
    {
      if (report) {
        report->reason = RejectReason::OutOfBounds;
      }
      return false;
    }
    bool has_new_cells = false;
    bool blocked = false;
    int best_priority = -1;
    double best_dist = std::numeric_limits<double>::infinity();
    unsigned int swept_cells = 0;
    unsigned int free_cells = 0;
    unsigned int unknown_cells = 0;
    for (unsigned int cy = y_min; cy <= y_max; ++cy) {
      for (unsigned int cx = x_min; cx <= x_max; ++cx) {
        double wx, wy;
        mapToWorld(cx, cy, wx, wy);
        const double dx = wx - x;
        const double dy = wy - y;
        const double along = dx * cos_yaw + dy * sin_yaw;
        const double across = -dx * sin_yaw + dy * cos_yaw;
        const double nearest = std::clamp(along, std::min(0.0, travel), std::max(0.0, travel));
        if (std::hypot(along - nearest, across) > extent) {
          continue;
        }
        const bool newly_swept = std::hypot(dx, dy) > extent;
        has_new_cells = has_new_cells || newly_swept;
        const unsigned char map_cost = map.getCost(cx, cy);
        const unsigned char clear_cost = getCost(cx, cy);
        if (newly_swept) {
          ++swept_cells;
          if (clear_cost == nav2_costmap_2d::FREE_SPACE) {
            ++free_cells;
          } else if (clear_cost == nav2_costmap_2d::NO_INFORMATION) {
            ++unknown_cells;
          }
        }
        RejectReason cell_reason = RejectReason::Cleared;
        int priority = 0;
        if (map_cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
          cell_reason = RejectReason::CostmapLethal;
          priority = 2;
        } else if (newly_swept && clear_cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
          cell_reason = RejectReason::CloudLethal;
          priority = 2;
        } else if (newly_swept && clear_cost != nav2_costmap_2d::FREE_SPACE) {
          cell_reason = RejectReason::Unobserved;
          priority = 1;
        }
        if (cell_reason != RejectReason::Cleared) {
          if (!report) {
            return false;
          }
          blocked = true;
          const double cell_dist = std::hypot(dx, dy);
          if (priority > best_priority || (priority == best_priority && cell_dist < best_dist)) {
            best_priority = priority;
            best_dist = cell_dist;
            const auto idx = getIndex(cx, cy);
            report->reason = cell_reason;
            report->block_x = wx;
            report->block_y = wy;
            report->block_dist = cell_dist;
            report->costmap_cost = map_cost;
            report->cloud_hits = hit_count_[idx];
            report->cloud_z = hit_z_[idx] >= 0.0f ? static_cast<double>(hit_z_[idx]) : 0.0;
          }
        }
      }
    }
    if (report) {
      report->swept_cells = swept_cells;
      report->free_cells = free_cells;
      report->unknown_cells = unknown_cells;
      if (blocked) {
        return false;
      }
      if (!has_new_cells) {
        report->reason = RejectReason::NoNewCells;
        return false;
      }
      return true;
    }
    return has_new_cells;
  }

private:
  void observeImpl(const tf2::Vector3 & origin, const tf2::Vector3 & hit, bool mark_obstacle)
  {
    constexpr double min_obstacle_height = 0.12;
    constexpr double max_obstacle_height = 1.35;
    const auto ray = hit - origin;
    const double length = std::hypot(ray.x(), ray.y());
    if (!std::isfinite(length) || !std::isfinite(hit.z()) ||
      length <= getResolution() || hit.z() < -0.10 || hit.z() > max_obstacle_height)
    {
      return;
    }
    unsigned int x, y, ox, oy;
    if (mark_obstacle && hit.z() >= min_obstacle_height && worldToMap(hit.x(), hit.y(), x, y)) {
      setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
      const auto hit_idx = getIndex(x, y);
      if (hit_count_[hit_idx] < std::numeric_limits<uint16_t>::max()) {
        ++hit_count_[hit_idx];
      }
      if (hit.z() > hit_z_[hit_idx]) {
        hit_z_[hit_idx] = static_cast<float>(hit.z());
      }
    }

    // Cover only cells short of a measured return, never beyond it.
    const auto limit = origin + ray * (std::min(1.0, length - getResolution()) / length);
    if (!worldToMap(origin.x(), origin.y(), ox, oy) ||
      !worldToMap(limit.x(), limit.y(), x, y))
    {
      return;
    }
    const auto index = getIndex(x, y);
    if (!traced_[index]) {
      traced_[index] = true;
      auto * cells = getCharMap();
      raytraceLine(
        [cells](unsigned int cell) {
          if (cells[cell] == nav2_costmap_2d::NO_INFORMATION) {
            cells[cell] = nav2_costmap_2d::FREE_SPACE;
          }
        }, ox, oy, x, y);
    }
  }

  std::vector<bool> traced_;
  std::vector<uint16_t> hit_count_;
  std::vector<float> hit_z_;
};

}  // namespace odin_nav_sim

#endif  // ODIN_NAV_SIM__TERMINAL_CLEARANCE_HPP_
