#include <algorithm>
#include <cmath>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/voxel_layer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace odin_nav_sim
{

/**
 * VoxelLayer with per-cell persistence TTL so transient upper-voxel returns
 * (e.g. walking pedestrians above ground-ray height) expire cleanly once
 * observations stop refreshing them.
 */
class DecayingVoxelLayer : public nav2_costmap_2d::VoxelLayer
{
public:
  void onInitialize() override
  {
    VoxelLayer::onInitialize();
    declareParameter("obstacle_persistence", rclcpp::ParameterValue(0.0));
    auto node = node_.lock();
    if (node) {
      node->get_parameter(name_ + ".obstacle_persistence", obstacle_persistence_);
    }
    last_marked_.assign(static_cast<size_t>(size_x_) * size_y_, -1.0);
  }

  void matchSize() override
  {
    VoxelLayer::matchSize();
    last_marked_.assign(static_cast<size_t>(size_x_) * size_y_, -1.0);
  }

  void reset() override
  {
    VoxelLayer::reset();
    last_marked_.assign(static_cast<size_t>(size_x_) * size_y_, -1.0);
  }

  void updateOrigin(double new_origin_x, double new_origin_y) override
  {
    const double old_ox = origin_x_;
    const double old_oy = origin_y_;
    VoxelLayer::updateOrigin(new_origin_x, new_origin_y);
    if (last_marked_.empty() || resolution_ <= 0.0) {
      return;
    }
    const int cell_ox = static_cast<int>(std::llround((origin_x_ - old_ox) / resolution_));
    const int cell_oy = static_cast<int>(std::llround((origin_y_ - old_oy) / resolution_));
    std::vector<double> shifted(static_cast<size_t>(size_x_) * size_y_, -1.0);
    for (unsigned int ny = 0; ny < size_y_; ++ny) {
      const int oy = static_cast<int>(ny) + cell_oy;
      if (oy < 0 || oy >= static_cast<int>(size_y_)) {
        continue;
      }
      for (unsigned int nx = 0; nx < size_x_; ++nx) {
        const int ox = static_cast<int>(nx) + cell_ox;
        if (ox >= 0 && ox < static_cast<int>(size_x_)) {
          shifted[ny * size_x_ + nx] =
            last_marked_[static_cast<size_t>(oy) * size_x_ + ox];
        }
      }
    }
    last_marked_.swap(shifted);
  }

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override
  {
    std::vector<nav2_costmap_2d::Observation> marking_obs;
    if (enabled_ && obstacle_persistence_ > 0.0) {
      getMarkingObservations(marking_obs);
    }

    VoxelLayer::updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
    if (!enabled_ || obstacle_persistence_ <= 0.0) {
      return;
    }

    const size_t total = static_cast<size_t>(size_x_) * size_y_;
    if (last_marked_.size() != total) {
      last_marked_.assign(total, -1.0);
    }
    const double now = clock_->now().seconds();

    for (const auto & obs : marking_obs) {
      if (!obs.cloud_ || obs.cloud_->data.empty()) {
        continue;
      }
      const double min_r2 = obs.obstacle_min_range_ * obs.obstacle_min_range_;
      const double max_r2 = obs.obstacle_max_range_ * obs.obstacle_max_range_;
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*obs.cloud_, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*obs.cloud_, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*obs.cloud_, "z");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        const double px = *iter_x;
        const double py = *iter_y;
        const double pz = *iter_z;
        if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) {
          continue;
        }
        const double dx = px - obs.origin_.x;
        const double dy = py - obs.origin_.y;
        const double dz = pz - obs.origin_.z;
        const double r2 = dx * dx + dy * dy + dz * dz;
        if (r2 < min_r2 || r2 >= max_r2) {
          continue;
        }
        unsigned int mx, my, mz;
        if (worldToMap3D(px, py, pz, mx, my, mz)) {
          last_marked_[static_cast<size_t>(my) * size_x_ + mx] = now;
        }
      }
    }

    for (size_t idx = 0; idx < total; ++idx) {
      if (costmap_[idx] == nav2_costmap_2d::LETHAL_OBSTACLE) {
        if (last_marked_[idx] < 0.0) {
          last_marked_[idx] = now;
        } else if (now - last_marked_[idx] > obstacle_persistence_) {
          voxel_grid_.clearVoxelColumn(static_cast<unsigned int>(idx));
          costmap_[idx] = nav2_costmap_2d::FREE_SPACE;
          last_marked_[idx] = -1.0;
          const unsigned int mx = static_cast<unsigned int>(idx % size_x_);
          const unsigned int my = static_cast<unsigned int>(idx / size_x_);
          double wx, wy;
          mapToWorld(mx, my, wx, wy);
          touch(wx, wy, min_x, min_y, max_x, max_y);
        }
      } else {
        last_marked_[idx] = -1.0;
      }
    }
  }

private:
  double obstacle_persistence_{0.0};
  std::vector<double> last_marked_;
};

}  // namespace odin_nav_sim

PLUGINLIB_EXPORT_CLASS(odin_nav_sim::DecayingVoxelLayer, nav2_costmap_2d::Layer)
