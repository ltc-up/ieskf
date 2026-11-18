// SPDX-License-Identifier: MIT
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/points/point_cloud.hpp>

#include "gicp15d/local_covariance.h"
#include "gicp15d/types.h"

namespace gicp15d {

/// One immutable target representation: points plus optional cropped tree.
struct TargetSet
{
  std::string path;
  std::string sha256;
  small_gicp::PointCloud::Ptr cloud;   ///< full map (or cropped when `cropped`)
  std::shared_ptr<KdTree> tree;
  bool cropped = false;
  std::string roi_key;
  std::size_t points = 0;
  std::size_t points_full = 0;
  double load_ms = 0.0;
  double covariance_ms = 0.0;
  double roi_ms = 0.0;
  double tree_ms = 0.0;
  CovarianceStats covariance_stats;
};

/// Owns the static global map and the ROI-cropped derivatives.
///
/// The covariance is estimated once on the full map (so a cropped ROI never
/// changes a point's local neighbourhood), and the crop only restricts the
/// nearest-neighbour search domain. That is a deliberate difference from the
/// Phase AW service, which searched the full map and then discarded
/// out-of-region candidates without retrying the next in-region neighbour.
class GlobalMapManager
{
public:
  GlobalMapManager(
    CovarianceMode mode, int covariance_knn, double floor_ratio, int num_threads,
    std::size_t max_points, bool rehash_on_lookup = true);

  /// Load and preprocess the target. Repeated calls with the same path and
  /// sha256 are served from the cache. Returns nullptr and fills `error`.
  std::shared_ptr<const TargetSet> load(
    const std::string & path, std::string * error, bool * cache_hit = nullptr);

  /// Crop the cached full target to the region. Cached per generation.
  std::shared_ptr<const TargetSet> crop(
    const std::shared_ptr<const TargetSet> & full, const ActiveRegion & region,
    std::string * error);

  void setRehashOnLookup(bool on) {rehash_on_lookup_ = on;}
  std::size_t cachedTargets() const {return targets_.size();}
  void clear();

private:
  struct Entry
  {
    std::string sha;
    std::shared_ptr<TargetSet> full;
  };
  CovarianceMode mode_;
  int covariance_knn_;
  double floor_ratio_;
  int num_threads_;
  std::size_t max_points_;
  /// Streaming services load one target once; re-hashing a 10 MB file per
  /// request would dominate the matcher budget. Off only for that use case.
  bool rehash_on_lookup_;
  std::vector<Entry> targets_;                         ///< path -> full set
  std::vector<std::shared_ptr<TargetSet>> crops_;      ///< (path, generation) -> crop
};

}  // namespace gicp15d
