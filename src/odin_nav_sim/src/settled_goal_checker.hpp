#pragma once
#include <utility>
#include "nav2_controller/plugins/simple_goal_checker.hpp"

namespace odin_nav_sim
{
class SettledGoalChecker : public nav2_controller::SimpleGoalChecker
{
public:
  void reset() override
  {
    SimpleGoalChecker::reset();
    reached_ = false;
  }

  bool isGoalReached(
    const geometry_msgs::msg::Pose &, const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::Twist &) override
  {
    // Humble calls this after computeVelocityCommands, with an independently transformed goal.
    return std::exchange(reached_, false);
  }

  void setReached(bool reached) {reached_ = reached;}

private:
  bool reached_{false};
};

}  // namespace odin_nav_sim
