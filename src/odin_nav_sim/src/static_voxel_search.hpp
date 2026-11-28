#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <small_gicp/ann/gaussian_voxelmap.hpp>

namespace odin_nav_sim
{

// The reference map is immutable during localization. Precompute the occupied
// neighbors of each query cell once and retain small_gicp's order for equal distances.
class StaticVoxelSearch
{
public:
  explicit StaticVoxelSearch(small_gicp::GaussianVoxelMap::ConstPtr map)
  : map_(std::move(map))
  {
    std::vector<Eigen::Vector3i> offsets;
    for (const auto & offset : map_->search_offsets) {
      if (std::find(offsets.begin(), offsets.end(), offset) != offsets.end()) {
        continue;
      }
      offsets.push_back(offset);
      for (size_t id = 0; id < map_->size(); ++id) {
        const auto & coord = map_->flat_voxels[id]->first.coord;
        neighbors_[coord - offset].push_back(static_cast<uint32_t>(id));
      }
    }
  }

  size_t nearest(const Eigen::Vector4d & query, size_t * index, double * distance) const
  {
    const Eigen::Vector3i cell = small_gicp::fast_floor(query * map_->inv_leaf_size).head<3>();
    const auto found = neighbors_.find(cell);
    *index = std::numeric_limits<size_t>::max();
    *distance = std::numeric_limits<double>::max();
    if (found == neighbors_.end()) {
      return 0;
    }
    for (const auto id : found->second) {
      const double d = (map_->flat_voxels[id]->second.mean - query).squaredNorm();
      if (d < *distance) {
        *index = map_->calc_index(id, 0);
        *distance = d;
      }
    }
    return *index != std::numeric_limits<size_t>::max();
  }

private:
  small_gicp::GaussianVoxelMap::ConstPtr map_;
  std::unordered_map<Eigen::Vector3i, std::vector<uint32_t>, small_gicp::XORVector3iHash> neighbors_;
};

}  // namespace odin_nav_sim

namespace small_gicp::traits
{
template<>
struct Traits<odin_nav_sim::StaticVoxelSearch>
{
  static size_t nearest_neighbor_search(
    const odin_nav_sim::StaticVoxelSearch & search, const Eigen::Vector4d & query,
    size_t * index, double * distance)
  {
    return search.nearest(query, index, distance);
  }
};
}  // namespace small_gicp::traits
