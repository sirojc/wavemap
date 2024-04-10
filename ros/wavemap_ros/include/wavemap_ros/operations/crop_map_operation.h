#ifndef WAVEMAP_ROS_OPERATIONS_CROP_MAP_OPERATION_H_
#define WAVEMAP_ROS_OPERATIONS_CROP_MAP_OPERATION_H_

#include <memory>
#include <string>
#include <utility>

#include <wavemap/config/config_base.h>
#include <wavemap/data_structure/volumetric/volumetric_data_structure_base.h>

#include "wavemap_ros/tf_transformer.h"

namespace wavemap {

class CropMapOperation {
 public:
  CropMapOperation() {}

  void run(const VolumetricDataStructureBase::Ptr occupancy_map, Point3D center_point, FloatingPoint max_considered_distance);

 private:

  template <typename MapT, typename IndicatorFunctionT>
  void eraseBlockIf(MapT* map, IndicatorFunctionT indicator_fn) {
    auto& block_map = map->getBlocks();
    for (auto block_it = block_map.begin(); block_it != block_map.end();) {
      const auto& block_index = block_it->first;
      if (std::invoke(indicator_fn, block_index)) {
        block_it = block_map.erase(block_it);
      } else {
        ++block_it;
      }
    }
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_OPERATIONS_CROP_MAP_OPERATION_H_
