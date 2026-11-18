// SPDX-License-Identifier: MIT
#pragma once

#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/points/point_cloud.hpp>

#include <cstddef>
#include <vector>

#include "gicp15d/types.h"

namespace gicp15d {

using KdTree = small_gicp::KdTree<small_gicp::PointCloud>;

struct CovarianceStats
{
  std::size_t points = 0;
  std::size_t insufficient_neighbours = 0;
  double lambda_max_median = 0.0;
  double lambda_min_median = 0.0;
  double effective_floor_median = 0.0;
  double condition_median = 0.0;
};

/// Estimate per-point normals and 3x3 covariances in place.
///
/// SMALL_GICP_UNIT reproduces upstream `estimate_covariances_omp` exactly:
/// biased sample covariance over the k nearest neighbours (the point itself is
/// included, as upstream does), eigenvectors kept, eigenvalues replaced by
/// (1e-3, 1, 1). Points with fewer than 5 neighbours get an isotropic identity
/// covariance, matching upstream's set_invalid.
///
/// SPECTRUM_FLOOR keeps the measured eigenvalues and floors each of them at
/// `floor_ratio * lambda_max`, so the metric retains the local spatial scale
/// instead of forcing every neighbourhood to unit variance.
CovarianceStats estimateCovariances(
  small_gicp::PointCloud & cloud, KdTree & tree, int num_neighbors,
  CovarianceMode mode, double floor_ratio, int num_threads);

/// Normals only (same neighbourhoods and orientation convention as upstream).
std::size_t estimateNormals(small_gicp::PointCloud & cloud, KdTree & tree, int num_neighbors,
  int num_threads);

/// Copy points + normals + covariances for the given index subset. No kd-tree,
/// no covariance recomputation: used to build ROI-cropped target clouds.
small_gicp::PointCloud::Ptr subsetCloud(
  const small_gicp::PointCloud & cloud, const std::vector<std::size_t> & indices);

}  // namespace gicp15d
