// SPDX-License-Identifier: MIT
#include "gicp15d/global_map_manager.h"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <small_gicp/ann/kdtree_omp.hpp>

#include "gicp15d/point_cloud_io.h"

namespace gicp15d {
namespace {

using Clock = std::chrono::steady_clock;
double msSince(Clock::time_point t)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - t).count();
}

}  // namespace

GlobalMapManager::GlobalMapManager(
  CovarianceMode mode, int covariance_knn, double floor_ratio, int num_threads,
  std::size_t max_points)
: mode_(mode), covariance_knn_(covariance_knn), floor_ratio_(floor_ratio),
  num_threads_(num_threads), max_points_(max_points)
{
}

std::shared_ptr<const TargetSet> GlobalMapManager::load(
  const std::string & path, std::string * error, bool * cache_hit)
{
  if (cache_hit) {*cache_hit = false;}
  for (auto & e : targets_) {
    if (e.full && e.full->path == path) {
      // Identity is the file content, not the path: a replaced file must not be
      // silently served from the cache.
      const std::string sha = fileSha256(path);
      if (!sha.empty() && sha == e.sha) {
        if (cache_hit) {*cache_hit = true;}
        return e.full;
      }
    }
  }

  const auto t0 = Clock::now();
  auto cloud = loadXyzBinChecked(path, 100, max_points_, error);
  if (!cloud) {return nullptr;}
  auto set = std::make_shared<TargetSet>();
  set->path = path;
  set->load_ms = msSince(t0);
  set->sha256 = fileSha256(path);
  if (set->sha256.empty()) {
    if (error) {*error = "TARGET_SHA256_FAILED";}
    return nullptr;
  }
  set->cloud = cloud;
  set->points = cloud->size();
  set->points_full = cloud->size();

  const auto ttree = Clock::now();
  set->tree = std::make_shared<KdTree>(cloud, small_gicp::KdTreeBuilderOMP(num_threads_));
  set->tree_ms = msSince(ttree);

  const auto tcov = Clock::now();
  set->covariance_stats = estimateCovariances(
    *cloud, *set->tree, covariance_knn_, mode_, floor_ratio_, num_threads_);
  set->covariance_ms = msSince(tcov);

  targets_.push_back({set->sha256, set});
  return set;
}

std::shared_ptr<const TargetSet> GlobalMapManager::crop(
  const std::shared_ptr<const TargetSet> & full, const ActiveRegion & region,
  std::string * error)
{
  if (!full) {
    if (error) {*error = "NULL_TARGET";}
    return nullptr;
  }
  if (!region.enabled) {return full;}
  for (const auto & c : crops_) {
    if (c && c->cropped && c->path == full->path &&
      c->roi_key == full->sha256 + "#" + std::to_string(region.generation))
    {
      return c;
    }
  }

  const auto t0 = Clock::now();
  std::vector<std::size_t> keep;
  keep.reserve(full->points);
  const double r2 = region.radius * region.radius;
  for (std::size_t i = 0; i < full->cloud->size(); i++) {
    const Eigen::Vector4d & p = full->cloud->point(i);
    const double dx = p.x() - region.center.x();
    const double dy = p.y() - region.center.y();
    if (dx * dx + dy * dy > r2) {continue;}
    if (p.z() < region.low_z || p.z() > region.high_z) {continue;}
    keep.push_back(i);
  }
  if (keep.size() < 100) {
    if (error) {*error = "ROI_TOO_FEW_TARGET_POINTS";}
    return nullptr;
  }
  auto set = std::make_shared<TargetSet>();
  set->path = full->path;
  set->sha256 = full->sha256;
  set->cropped = true;
  set->roi_key = full->sha256 + "#" + std::to_string(region.generation);
  set->points = keep.size();
  set->roi_ms = msSince(t0);

  const auto tcrop = Clock::now();
  set->cloud = subsetCloud(*full->cloud, keep);
  set->roi_ms = msSince(tcrop);

  const auto ttree = Clock::now();
  set->tree = std::make_shared<KdTree>(set->cloud, small_gicp::KdTreeBuilderOMP(num_threads_));
  set->tree_ms = msSince(ttree);
  crops_.push_back(set);
  return set;
}

void GlobalMapManager::clear()
{
  targets_.clear();
  crops_.clear();
}

}  // namespace gicp15d
