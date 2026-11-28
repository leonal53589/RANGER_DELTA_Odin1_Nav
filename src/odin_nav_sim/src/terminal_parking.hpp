#ifndef ODIN_NAV_SIM__TERMINAL_PARKING_HPP_
#define ODIN_NAV_SIM__TERMINAL_PARKING_HPP_

#include <cmath>
#include <optional>

namespace odin_nav_sim
{

constexpr double kRepositionToleranceFactor = 0.65;

class TerminalParking
{
public:
  enum class Command {FollowPath, AlignGoal, Hold, Reposition, Reached, Failed};

  struct Sample
  {
    double time;
    double distance;
    double heading_error;
    double xy_tolerance;
    double yaw_tolerance;
    bool stopped;
    bool fresh;
  };

  TerminalParking() = default;
  TerminalParking(double settle_hold_seconds, double settle_timeout_seconds)
  : settle_hold_seconds_(settle_hold_seconds),
    settle_timeout_seconds_(settle_timeout_seconds)
  {}

  bool isRepositioning() const {return phase_ == Phase::Reposition;}

  Command update(const Sample & sample)
  {
    if (started_ && sample.time - *started_ >= 30.0) {
      return Command::Failed;
    }
    if (!sample.fresh) {
      stable_since_.reset();
      return Command::Hold;
    }

    const bool xy_ok = sample.distance <= sample.xy_tolerance;
    const bool yaw_ok = std::abs(sample.heading_error) < sample.yaw_tolerance;
    if (phase_ == Phase::Approach) {
      if (!xy_ok) {
        return Command::FollowPath;
      }
      started_ = sample.time;
      phase_ = Phase::Align;
    }
    if (phase_ == Phase::Reposition) {
      // Reserve an inner margin only while a final turn is still needed. If both
      // errors already pass, brake and verify the full stationary hold.
      if (!(xy_ok && yaw_ok) &&
        sample.distance > kRepositionToleranceFactor * sample.xy_tolerance)
      {
        return Command::Reposition;
      }
      phase_ = Phase::Align;
    }
    if (phase_ == Phase::Align) {
      if (!yaw_ok && sample.distance <= sample.xy_tolerance + 0.05) {
        return Command::AlignGoal;
      }
      phase_ = Phase::Settle;
      settle_started_ = sample.time;
      stable_since_.reset();
    }

    if (sample.time - settle_started_ >= settle_timeout_seconds_) {
      return Command::Failed;
    }
    if (!sample.stopped) {
      stable_since_.reset();
      return Command::Hold;
    }
    const auto condition = !xy_ok ? Condition::Offset :
      (!yaw_ok ? Condition::Heading : Condition::Reached);
    if (!stable_since_ || condition != condition_) {
      condition_ = condition;
      stable_since_ = sample.time;
      return Command::Hold;
    }
    if (sample.time - *stable_since_ < settle_hold_seconds_) {
      return Command::Hold;
    }
    if (condition == Condition::Reached) {
      return Command::Reached;
    }
    if (corrections_ >= 2) {
      return Command::Failed;
    }

    ++corrections_;
    phase_ = condition == Condition::Offset ? Phase::Reposition : Phase::Align;
    stable_since_.reset();
    return Command::Hold;
  }

private:
  enum class Phase {Approach, Align, Settle, Reposition};
  enum class Condition {Offset, Heading, Reached};

  Phase phase_{Phase::Approach};
  Condition condition_{Condition::Reached};
  std::optional<double> started_;
  std::optional<double> stable_since_;
  double settle_hold_seconds_{0.30};
  double settle_timeout_seconds_{3.0};
  double settle_started_{0.0};
  int corrections_{0};
};

}  // namespace odin_nav_sim

#endif  // ODIN_NAV_SIM__TERMINAL_PARKING_HPP_
