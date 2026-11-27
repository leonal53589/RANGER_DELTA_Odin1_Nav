#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace odin_nav_sim
{

struct StationOffset
{
  double x, y, yaw;  // Measured base pose in A's axes, metres and radians.
};

struct StationCommand
{
  double vx{}, vy{}, wz{};
  bool reached{};
};

class StationServo
{
public:
  void pause()
  {
    axis_ = -1;
    first_still_ = -1;
  }

  // Each short segment starts and ends at rest. A new scan also controls every moving tick.
  StationCommand update(double now, double stamp, StationOffset p, bool still)
  {
    if (stamp <= 0 || !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.yaw) ||
      std::hypot(p.x, p.y) > 0.30 || std::abs(p.yaw) > 0.44)
    {
      throw std::runtime_error("station measurement outside capture envelope");
    }
    if (stamp != last_stamp_) {
      if (last_stamp_ > 0 && (stamp < last_stamp_ ||
        std::hypot(p.x - previous_.x, p.y - previous_.y) > 0.025 ||
        std::abs(p.yaw - previous_.yaw) > 0.05))
      {
        throw std::runtime_error("discontinuous station measurement");
      }
      last_stamp_ = stamp;
      previous_ = p;
    }
    const double c = std::cos(p.yaw), s = std::sin(p.yaw);
    // Translation of A's origin expressed in the current body (inverse of measured SE(2)).
    const double dx = -c * p.x - s * p.y, dy = s * p.x - c * p.y;
    const double errors[] = {dx, dy, -p.yaw};
    if (axis_ < 0) {
      if (!still) {
        first_still_ = -1;
        return {};
      }
      if (first_still_ < 0) {
        first_still_ = stamp;
        first_pose_ = p;
        return {};
      }
      if (std::hypot(p.x - first_pose_.x, p.y - first_pose_.y) > 0.008 ||
        std::abs(p.yaw - first_pose_.yaw) > 0.02)
      {
        first_still_ = stamp;
        first_pose_ = p;
      }
      if (stamp - first_still_ < 0.45) {return {};}
      if (std::abs(p.yaw) > yaw_tol) {axis_ = 2;}
      else if (std::hypot(dx, dy) > xy_tol) {axis_ = std::abs(dx) >= std::abs(dy) ? 0 : 1;}
      else {return {0, 0, 0, true};}
      if (++segments_ > 18) {throw std::runtime_error("station correction budget exhausted");}
      initial_error_ = errors[axis_];
      step_ = std::min(axis_ == 2 ? 0.105 : 0.05, std::abs(initial_error_));
      best_ = 0;
      progress_time_ = now;
    }
    const double sign = std::copysign(1.0, initial_error_);
    const double advance = sign * (initial_error_ - errors[axis_]);
    const double remaining = step_ - advance;
    if (remaining <= (axis_ == 2 ? 0.012 : 0.002) ||
      (axis_ == 2 ? std::abs(p.yaw) <= yaw_tol : std::hypot(dx, dy) <= xy_tol))
    {
      pause();
      return {};
    }
    if (advance > best_ + (axis_ == 2 ? 0.012 : 0.003)) {
      best_ = advance;
      progress_time_ = now;
    }
    if (advance < -(axis_ == 2 ? 0.04 : 0.015) || now - progress_time_ > 4.0) {
      throw std::runtime_error("station correction moving away or stalled");
    }
    const double speed = sign * (axis_ == 2 ?
      std::clamp(1.8 * remaining, 0.045, 0.12) : std::clamp(1.8 * remaining, 0.013, 0.03));
    return {axis_ == 0 ? speed : 0., axis_ == 1 ? speed : 0., axis_ == 2 ? speed : 0., false};
  }

  static constexpr double xy_tol = 0.010;
  static constexpr double yaw_tol = 2.0 * 3.141592653589793 / 180.0;

private:
  StationOffset previous_{}, first_pose_{};
  double first_still_{-1}, last_stamp_{}, initial_error_{}, step_{}, best_{}, progress_time_{};
  int axis_{-1}, segments_{};
};
}  // namespace odin_nav_sim
