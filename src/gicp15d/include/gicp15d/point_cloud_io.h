// SPDX-License-Identifier: MIT
#pragma once

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>

#include <small_gicp/points/point_cloud.hpp>

namespace gicp15d {

using Cloud = small_gicp::PointCloud;
using CloudPtr = std::shared_ptr<Cloud>;

/// Point file contract used by every frozen corpus in this study:
///   little-endian uint64 n, followed by n * 3 little-endian float32 (x, y, z).
/// This is the historical `captured_sources` / `map_target.bin` format.
/// Returns nullptr and fills `error` on failure; never throws on bad data.
CloudPtr loadXyzBin(const std::string & path, std::string * error);

/// Convenience: load, then reject if the point count is outside [min,max].
CloudPtr loadXyzBinChecked(
  const std::string & path, std::size_t min_points, std::size_t max_points,
  std::string * error);

/// sha256 of a file, lowercase hex. Returns empty string on IO failure.
std::string fileSha256(const std::string & path);

/// Voxel-downsample a cloud by centroid, producing a new cloud with points only
/// (no normals/covariances). Used only for the explicit coarse-to-fine pass.
CloudPtr voxelDownsample(const Cloud & cloud, double leaf);

// ---- cloud statistics used by diagnostics and tests ----
struct CloudStats
{
  std::size_t n = 0;
  Eigen::Vector3d min = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d max = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  std::size_t nonfinite = 0;
};
CloudStats cloudStats(const Cloud & cloud);

}  // namespace gicp15d
