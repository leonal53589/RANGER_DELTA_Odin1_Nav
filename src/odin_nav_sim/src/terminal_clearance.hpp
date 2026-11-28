#ifndef ODIN_NAV_SIM__TERMINAL_CLEARANCE_HPP_
#define ODIN_NAV_SIM__TERMINAL_CLEARANCE_HPP_

#include <algorithm>
#include <cmath>
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
  explicit TerminalClearance(const nav2_costmap_2d::Costmap2D & map)
  : Costmap2D(map.getSizeInCellsX(), map.getSizeInCellsY(), map.getResolution(),
      map.getOriginX(), map.getOriginY(), nav2_costmap_2d::NO_INFORMATION),
    traced_(static_cast<size_t>(map.getSizeInCellsX()) * map.getSizeInCellsY(), false)
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
    double radius) const
  {
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
      return false;
    }
    bool has_new_cells = false;
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
        if (map.getCost(cx, cy) >= nav2_costmap_2d::LETHAL_OBSTACLE ||
          (newly_swept && getCost(cx, cy) != nav2_costmap_2d::FREE_SPACE))
        {
          return false;
        }
      }
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
};

}  // namespace odin_nav_sim

#endif  // ODIN_NAV_SIM__TERMINAL_CLEARANCE_HPP_
