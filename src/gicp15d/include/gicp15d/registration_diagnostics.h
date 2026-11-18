// SPDX-License-Identifier: MIT
#pragma once

#include <Eigen/Core>

#include <vector>

#include <small_gicp/points/point_cloud.hpp>

#include "gicp15d/correspondence_rejector.h"
#include "gicp15d/local_covariance.h"
#include "gicp15d/robust_gicp_factor.h"
#include "gicp15d/types.h"

namespace gicp15d {

/// Fresh linearisation at the FINAL pose. Nothing is reused from the optimizer:
/// the optimizer's stored H/b belong to the pre-step pose, and the factor's
/// Mahalanobis term is frozen at its own linearisation pose, so a post-hoc
/// refresh is the only way to report H at the pose that is actually returned.
void computeDiagnostics(
  const small_gicp::PointCloud & target,
  const small_gicp::PointCloud & source,
  const KdTree & target_tree,
  const GicpRejector & rejector_template,
  const RobustGICPFactor::Setting & factor_setting,
  int num_threads,
  const Eigen::Isometry3d & T_final,
  ResidualStatistics * residuals,
  InformationDiagnostics * information);

/// Eigen-analysis of a symmetric 6x6 information matrix. Ordering follows
/// small_gicp's delta: [rotation(0..2), translation(3..5)].
InformationDiagnostics analyseInformation(
  const Eigen::Matrix<double, 6, 6> & H, const Eigen::Matrix<double, 6, 1> & b,
  double cost);

}  // namespace gicp15d
