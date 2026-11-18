// SPDX-License-Identifier: MIT
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <cstddef>

#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/ann/traits.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/points/traits.hpp>

#include "gicp15d/types.h"

namespace gicp15d {

/// Correspondence rejector combining three independent gates.
///
/// 1. bilateral distance gate (metres, compared squared as upstream does);
/// 2. optional active region (cylinder + Z band) in the map frame;
/// 3. optional reciprocal check: the target point, mapped back into the source
///    frame, must find the same source index as its nearest source neighbour.
///
/// Counters are mutable so the rejector can stay const as the reduction
/// interface requires. They are diagnostic only and are read after align().
struct GicpRejector
{
  struct Counters
  {
    std::size_t total = 0;
    std::size_t rejected_distance = 0;
    std::size_t rejected_region = 0;
    std::size_t rejected_reciprocal = 0;
  };

  double max_dist_sq = 16.0;
  ActiveRegion region;
  bool reciprocal_check = false;
  /// Set by the frontend before align when reciprocal_check is enabled.
  const small_gicp::KdTree<small_gicp::PointCloud> * source_tree = nullptr;
  mutable Counters counters;

  template<typename TargetPointCloud, typename SourcePointCloud>
  bool operator()(
    const TargetPointCloud & target, const SourcePointCloud & source,
    const Eigen::Isometry3d & T, std::size_t target_index, std::size_t source_index,
    double sq_dist) const
  {
    counters.total++;

    if (sq_dist > max_dist_sq) {
      counters.rejected_distance++;
      return true;
    }

    if (region.enabled) {
      const Eigen::Vector4d pt = small_gicp::traits::point(target, target_index);
      const double dx = pt.x() - region.center.x();
      const double dy = pt.y() - region.center.y();
      if (dx * dx + dy * dy > region.radius * region.radius ||
        pt.z() < region.low_z || pt.z() > region.high_z)
      {
        counters.rejected_region++;
        return true;
      }
    }

    if (reciprocal_check && source_tree) {
      // Map the target point back into the source frame and ask which source
      // point is closest to it. A consistent pair answers `source_index`.
      const Eigen::Vector4d back = T.inverse() * small_gicp::traits::point(target, target_index);
      std::size_t rev = 0;
      double rev_sq = 0.0;
      if (small_gicp::traits::nearest_neighbor_search(
          *source_tree, back, &rev, &rev_sq) && rev != source_index)
      {
        counters.rejected_reciprocal++;
        return true;
      }
    }

    return false;
  }
};

}  // namespace gicp15d
