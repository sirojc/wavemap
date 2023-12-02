#ifndef WAVEMAP_UTILS_NEIGHBORS_GRID_NEIGHBORHOOD_H_
#define WAVEMAP_UTILS_NEIGHBORS_GRID_NEIGHBORHOOD_H_

#include "wavemap/common.h"
#include "wavemap/utils/iterate/grid_iterator.h"
#include "wavemap/utils/math/int_math.h"
#include "wavemap/utils/neighbors/adjacency.h"
#include "wavemap/utils/neighbors/grid_adjacency.h"

namespace wavemap {
template <int dim>
struct GridNeighborhood {
  static constexpr int numNeighbors(AdjacencyType adjacency_type);
  static constexpr int numNeighbors(AdjacencyMask adjacency_mask);

  template <AdjacencyMask adjacency_mask>
  static std::array<Index<dim>,
                    GridNeighborhood<dim>::numNeighbors(adjacency_mask)>
  generateIndexOffsets();

  static auto generateIndexOffsetsAllDisjointAdjacent() {
    return generateIndexOffsets<kAdjacencyAnyDisjoint<dim>>();
  }
  static auto generateIndexOffsetsAllAdjacent() {
    return generateIndexOffsets<kAdjacencyAny>();
  }

  template <typename VectorT, size_t array_length>
  static constexpr std::array<FloatingPoint, array_length> computeOffsetLengths(
      const std::array<VectorT, array_length>& offsets, FloatingPoint scale);
};
}  // namespace wavemap

#include "wavemap/utils/neighbors/impl/grid_neighborhood_inl.h"

#endif  // WAVEMAP_UTILS_NEIGHBORS_GRID_NEIGHBORHOOD_H_