#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "nav2_core/controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "settled_goal_checker.hpp"
#include "station_servo.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace odin_nav_sim
{
using Pose = geometry_msgs::msg::PoseStamped;
using Odom = nav_msgs::msg::Odometry;
using Grid = nav_msgs::msg::OccupancyGrid;

class StationController : public nav2_core::Controller
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map) override
  {
    node_ = parent;
    auto node = parent.lock();
    map_ = map;
    tf_ = tf;
    nav2_util::declare_parameter_if_not_declared(node, name + ".target",
      rclcpp::ParameterValue(std::vector<double>{}));
    target_ = node->get_parameter(name + ".target").as_double_array();
    if (target_.size() != 3 || !std::all_of(target_.begin(), target_.end(),
      [](double v) {return std::isfinite(v);}))
    {
      throw std::invalid_argument("StationAlign requires the captured map pose");
    }
    sub_ = node->create_subscription<Pose>("/station_a/offset", 1,
      [this](Pose::ConstSharedPtr msg) {std::atomic_store(&observation_, msg);});
    odom_sub_ = node->create_subscription<Odom>("/odom", rclcpp::SensorDataQoS().keep_last(1),
      [this](Odom::ConstSharedPtr msg) {std::atomic_store(&odom_, msg);});
    grid_sub_ = node->create_subscription<Grid>("/global_costmap/costmap",
      rclcpp::QoS(1).transient_local(),
      [this](Grid::ConstSharedPtr msg) {std::atomic_store(&global_, msg);});
  }

  void cleanup() override {sub_.reset(); odom_sub_.reset(); grid_sub_.reset();}
  void activate() override {}
  void deactivate() override {}
  void setSpeedLimit(const double & limit, const bool & percent) override
  {
    // Nav2 reserves zero for NO_SPEED_LIMIT.
    scale_ = limit == 0 ? 1.0 : std::clamp(percent ? limit / 100.0 : limit / 0.03, 0., 1.);
  }

  void setPlan(const nav_msgs::msg::Path & path) override
  {
    if (path.poses.size() != 1 || path.header.frame_id != "map") {
      throw nav2_core::PlannerException("StationAlign requires the original station goal");
    }
    const auto & p = path.poses.front().pose;
    if (!(std::hypot(p.position.x - target_[0], p.position.y - target_[1]) <= 0.001 &&
      std::abs(std::remainder(tf2::getYaw(p.orientation) - target_[2], 2*M_PI)) <= 0.002))
    {
      throw nav2_core::PlannerException("Goal differs from the captured station");
    }
    servo_ = {};
    started_ = node_.lock()->now().seconds();
    last_valid_ = started_;
    received_ = false;
    failed_.clear();
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(const Pose & pose,
    const geometry_msgs::msg::Twist &, nav2_core::GoalChecker * checker) override
  {
    auto * settled = dynamic_cast<SettledGoalChecker *>(checker);
    if (!settled) {throw nav2_core::PlannerException("StationAlign requires SettledGoalChecker");}
    settled->setReached(false);
    auto node = node_.lock();
    auto now = node->now();
    geometry_msgs::msg::TwistStamped out;
    out.header.frame_id = "base_link";
    out.header.stamp = now;
    if (!failed_.empty()) {throw nav2_core::PlannerException(failed_);}
    const auto obs = std::atomic_load(&observation_);
    const auto odom = std::atomic_load(&odom_);
    auto fresh = [&](const auto & stamp, double max_age) {
        const auto age = (now - rclcpp::Time(stamp)).seconds();
        return age >= 0.0 && age <= max_age;
      };
    try {
      if (now.seconds() - started_ > 50) {throw std::runtime_error("station alignment timeout");}
      if (!obs || !odom || obs->header.frame_id != "station_a" ||
        odom->header.frame_id != "odom" || odom->child_frame_id != "base_link" ||
        !fresh(obs->header.stamp, 0.35) || !fresh(odom->header.stamp, 0.20) ||
        rclcpp::Time(obs->header.stamp).seconds() <= started_)
      {
        servo_.pause();
        const double wait_limit = received_ ? 1.8 : 4.0;
        if (now.seconds() - last_valid_ > wait_limit) {
          throw std::runtime_error("station lost valid wall observation or wheel feedback");
        }
        return out;
      }
      const StationOffset p{obs->pose.position.x, obs->pose.position.y, tf2::getYaw(obs->pose.orientation)};
      if (!received_) {
        RCLCPP_INFO(node->get_logger(), "Station A pre-alignment: %.3f %.3f m, %.2f deg",
          p.x, p.y, p.yaw * 180/M_PI);
      }
      received_ = true;
      last_valid_ = now.seconds();
      const auto & v = odom->twist.twist;
      const bool stopped = std::hypot(v.linear.x, v.linear.y) < 0.008 && std::abs(v.angular.z) < 0.016;
      auto cmd = servo_.update(now.seconds(), rclcpp::Time(obs->header.stamp).seconds(), p, stopped);
      if (cmd.reached) {
        settled->setReached(true);
        RCLCPP_INFO(node->get_logger(), "Station A verified at rest: %.3f %.3f m, %.2f deg",
          p.x, p.y, p.yaw * 180/M_PI);
      } else if (cmd.vx || cmd.vy || cmd.wz) {
        const auto grid = std::atomic_load(&global_);
        Pose global_pose;
        if (!grid || grid->header.frame_id != "map" || !fresh(grid->header.stamp, 2.) ||
          !nav2_util::getCurrentPose(global_pose, *tf_, "map", "base_link", 0.0) ||
          !clear(pose, global_pose, *grid, cmd))
        {
          throw std::runtime_error("station footprint sweep blocked or costmap unavailable");
        }
        out.twist.linear.x = scale_ * cmd.vx;
        out.twist.linear.y = scale_ * cmd.vy;
        out.twist.angular.z = scale_ * cmd.wz;
      }
      return out;
    } catch (const std::runtime_error & e) {
      failed_ = e.what();  // Controller server retries for failure_tolerance. Keep the stop latched.
      throw nav2_core::PlannerException(failed_);
    }
  }

private:
  bool clear(const Pose & local, const Pose & global, const Grid & grid, StationCommand cmd)
  {
    auto * costmap = map_->getCostmap();
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());
    const auto & g = grid.info;
    if (local.header.frame_id != map_->getGlobalFrameID() || !map_->isCurrent() ||
      !(g.resolution > 0) || grid.data.size() != size_t(g.width) * g.height ||
      std::abs(tf2::getYaw(g.origin.orientation)) > 1e-6) {return false;}
    const double speed = std::hypot(cmd.vx, cmd.vy);
    // The global and local inflation layers already include the circular footprint.
    // Check its centre through the entire next segment plus 3 cm braking margin.
    for (double d = 0.; d <= (speed ? 0.081 : 0.001); d += 0.01) {
      auto centre = [&](const Pose & in) {
          const double angle = tf2::getYaw(in.pose.orientation) + std::atan2(cmd.vy, cmd.vx);
          return std::make_pair(in.pose.position.x + d * std::cos(angle),
            in.pose.position.y + d * std::sin(angle));
        };
      const auto [lx, ly] = centre(local);
      unsigned int ix, iy;
      if (!costmap->worldToMap(lx, ly, ix, iy) ||
        costmap->getCost(ix, iy) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {return false;}
      const auto [gx, gy] = centre(global);
      const int x = std::floor((gx - g.origin.position.x) / g.resolution);
      const int y = std::floor((gy - g.origin.position.y) / g.resolution);
      if (x < 0 || y < 0 || unsigned(x) >= g.width || unsigned(y) >= g.height) {return false;}
      const int cost = grid.data[size_t(y) * g.width + x];
      if (cost < 0 || cost >= 99) {return false;}
    }
    return true;
  }

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Subscription<Pose>::SharedPtr sub_;
  rclcpp::Subscription<Odom>::SharedPtr odom_sub_;
  rclcpp::Subscription<Grid>::SharedPtr grid_sub_;
  Pose::ConstSharedPtr observation_;
  Odom::ConstSharedPtr odom_;
  Grid::ConstSharedPtr global_;
  std::vector<double> target_;
  std::atomic<double> scale_{1.};
  double started_{}, last_valid_{};
  bool received_{};
  std::string failed_;
  StationServo servo_;
};
}  // namespace odin_nav_sim

PLUGINLIB_EXPORT_CLASS(odin_nav_sim::StationController, nav2_core::Controller)
