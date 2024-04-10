#include "wavemap_ros/operations/crop_map_operation.h"

#include <wavemap/data_structure/volumetric/hashed_blocks.h>
#include <wavemap/data_structure/volumetric/hashed_chunked_wavelet_octree.h>
#include <wavemap/data_structure/volumetric/hashed_wavelet_octree.h>

namespace wavemap {

void CropMapOperation::run(VolumetricDataStructureBase::Ptr occupancy_map,
                           Point3D center_point, FloatingPoint max_considered_distance) {
  // If the map is empty, there's no work to do
  if (occupancy_map->empty()) {
    ROS_ERROR("occupancy map is empt, nothing to crop");
    return;
  }

  const IndexElement tree_height = occupancy_map->getTreeHeight();
  const FloatingPoint min_cell_width = occupancy_map->getMinCellWidth();

  auto indicator_fn = [tree_height, min_cell_width, max_considered_distance, center_point](const Index3D& block_index) {
    const auto block_node_index = OctreeIndex{tree_height, block_index};
    const auto block_aabb = convert::nodeIndexToAABB(block_node_index, min_cell_width);
    const FloatingPoint d_B_block = block_aabb.minDistanceTo(center_point);
    return max_considered_distance < d_B_block;
  };

  if (auto* hashed_blocks =
          dynamic_cast<HashedWaveletOctree*>(occupancy_map.get());
      hashed_blocks) {
    eraseBlockIf(hashed_blocks, indicator_fn);
  } else if (auto* hashed_wavelet_octree =
                 dynamic_cast<HashedWaveletOctree*>(occupancy_map.get());
             hashed_wavelet_octree) {
    eraseBlockIf(hashed_wavelet_octree, indicator_fn);
  } else if (auto* hashed_chunked_wavelet_octree =
                 dynamic_cast<HashedChunkedWaveletOctree*>(
                     occupancy_map.get());
             hashed_chunked_wavelet_octree) {
    eraseBlockIf(hashed_chunked_wavelet_octree, indicator_fn);
  } else {
    ROS_WARN(
        "Map cropping is only supported for hash-based map data structures.");
  }

  return;
}
}  // namespace wavemap
