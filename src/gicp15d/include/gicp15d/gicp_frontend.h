// SPDX-License-Identifier: MIT
#pragma once

#include <memory>
#include <string>

#include "gicp15d/correspondence_rejector.h"
#include "gicp15d/global_map_manager.h"
#include "gicp15d/types.h"

namespace gicp15d {

/// Single-frame GICP global registration frontend.
///
/// Owns the static global map cache and runs one alignment per request. The
/// returned transform is exactly what the solver produced: nothing is selected,
/// substituted, or corrected using any reference trajectory.
class GicpFrontend
{
public:
  explicit GicpFrontend(const FrontendConfig & config);

  RegistrationOutput align(const RegistrationRequest & request);

  const FrontendConfig & config() const {return config_;}
  std::string configJson() const;

  /// Named configurations. Unknown names throw std::invalid_argument.
  static FrontendConfig preset(const std::string & name);

  void clearCaches();

private:
  struct AlignPass
  {
    Eigen::Isometry3d final = Eigen::Isometry3d::Identity();
    bool converged = false;
    int iterations = 0;
    std::size_t inliers = 0;
    double cost = 0.0;
    GicpRejector::Counters counters;
    double align_ms = 0.0;
  };

  AlignPass runPass(
    const small_gicp::PointCloud & target, const KdTree & target_tree,
    const small_gicp::PointCloud & source, const KdTree & source_tree,
    const ActiveRegion & region, const Eigen::Isometry3d & init,
    int max_iterations, double max_corr, bool use_robust) const;

  RegistrationOutput probe(
    const RegistrationRequest & request, const small_gicp::PointCloud & target,
    const KdTree & target_tree, const small_gicp::PointCloud & source,
    const KdTree & source_tree, double offset_m, const Eigen::Vector3d & axis);

  FrontendConfig config_;
  GlobalMapManager maps_;

  std::shared_ptr<const TargetSet> coarse_target_;
  std::string coarse_target_key_;
  std::shared_ptr<const TargetSet> roi_target_;
  std::string roi_target_key_;
};

/// Serialise a FrontendConfig as a single-line JSON object (run ledger).
std::string frontendConfigJson(const FrontendConfig & config);
const char * covarianceModeName(CovarianceMode mode);
const char * robustKernelName(RobustKernel kernel);

}  // namespace gicp15d
