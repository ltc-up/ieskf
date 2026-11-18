// SPDX-License-Identifier: MIT
#include "gicp15d/local_covariance.h"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <limits>
#include <vector>

namespace gicp15d {
namespace {

struct Feature
{
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
  bool ok = false;
  bool insufficient = false;
  double lmax = 1.0;
  double lmin = 1.0;
};

Feature computeFeature(
  const small_gicp::PointCloud & cloud, const std::size_t * indices, int n,
  CovarianceMode mode, double floor_ratio)
{
  Feature f;
  if (n < 5) {
    // Upstream CovarianceSetter::set_invalid: Identity with cov(3,3)=0.
    // That is isotropic unit covariance, i.e. maximum unjustified confidence.
    f.cov = Eigen::Matrix3d::Identity();
    f.normal = Eigen::Vector3d::Zero();
    f.insufficient = true;
    return f;
  }

  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  Eigen::Matrix3d sum_cross = Eigen::Matrix3d::Zero();
  for (int i = 0; i < n; i++) {
    const Eigen::Vector3d p = cloud.point(indices[i]).head<3>();
    sum += p;
    sum_cross += p * p.transpose();
  }
  // Upstream form: (sum_cross - mean * sum_points^T) / n   (biased, no Bessel).
  const Eigen::Vector3d mean = sum / double(n);
  const Eigen::Matrix3d cov = (sum_cross - mean * sum.transpose()) / double(n);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
  if (es.info() != Eigen::Success) {
    f.cov = Eigen::Matrix3d::Identity();
    f.insufficient = true;
    return f;
  }
  const Eigen::Matrix3d V = es.eigenvectors();
  const Eigen::Vector3d l = es.eigenvalues();  // ascending

  f.normal = V.col(0).normalized();
  // Upstream NormalSetter convention, evaluated at index 0 of the query point.
  if (cloud.point(indices[0]).head<3>().dot(f.normal) > 0) {
    f.normal = -f.normal;
  }

  Eigen::Vector3d values;
  if (mode == CovarianceMode::SMALL_GICP_UNIT) {
    values << 1.0e-3, 1.0, 1.0;
  } else {
    const double lmax = std::max(l(2), 1.0e-12);
    const double floor_abs = std::max(floor_ratio, 0.0) * lmax;
    values << std::max(l(0), floor_abs), std::max(l(1), floor_abs), lmax;
  }
  f.cov = V * values.asDiagonal() * V.transpose();
  f.lmax = values(2);
  f.lmin = values(0);
  f.ok = true;
  return f;
}

double medianOf(std::vector<double> v)
{
  if (v.empty()) {return std::numeric_limits<double>::quiet_NaN();}
  const std::size_t mid = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(mid), v.end());
  return v[mid];
}

}  // namespace

CovarianceStats estimateCovariances(
  small_gicp::PointCloud & cloud, KdTree & tree, int num_neighbors,
  CovarianceMode mode, double floor_ratio, int num_threads)
{
  CovarianceStats stats;
  stats.points = cloud.size();
  if (cloud.empty()) {return stats;}
  const int k = std::max(5, num_neighbors);
  const int threads = std::max(1, num_threads);

  std::vector<double> lmax;
  std::vector<double> lmin;
  lmax.reserve(cloud.size());
  lmin.reserve(cloud.size());

#pragma omp parallel num_threads(threads)
  {
    std::vector<std::size_t> indices(static_cast<std::size_t>(k));
    std::vector<double> sq_dists(static_cast<std::size_t>(k));
    std::size_t local_insufficient = 0;
    std::vector<double> local_lmax;
    std::vector<double> local_lmin;
    local_lmax.reserve(cloud.size() / static_cast<std::size_t>(threads) + 64);
    local_lmin.reserve(cloud.size() / static_cast<std::size_t>(threads) + 64);

#pragma omp for schedule(guided, 16)
    for (std::int64_t i = 0; i < static_cast<std::int64_t>(cloud.size()); i++) {
      const std::size_t idx = static_cast<std::size_t>(i);
      const int n = static_cast<int>(tree.knn_search(
          cloud.point(idx), k, indices.data(), sq_dists.data()));
      const Feature f = computeFeature(cloud, indices.data(), n, mode, floor_ratio);
      cloud.normal(idx) << f.normal.x(), f.normal.y(), f.normal.z(), 0.0;
      cloud.cov(idx).setZero();
      cloud.cov(idx).block<3, 3>(0, 0) = f.cov;
      if (f.insufficient) {
        local_insufficient++;
      } else {
        local_lmax.push_back(f.lmax);
        local_lmin.push_back(f.lmin);
      }
    }

#pragma omp critical
    {
      stats.insufficient_neighbours += local_insufficient;
      lmax.insert(lmax.end(), local_lmax.begin(), local_lmax.end());
      lmin.insert(lmin.end(), local_lmin.begin(), local_lmin.end());
    }
  }

  stats.lambda_max_median = medianOf(lmax);
  stats.lambda_min_median = medianOf(lmin);
  if (std::isfinite(stats.lambda_max_median) && stats.lambda_max_median > 0) {
    stats.effective_floor_median = stats.lambda_min_median / stats.lambda_max_median;
    stats.condition_median = stats.lambda_max_median / stats.lambda_min_median;
  }
  return stats;
}

std::size_t estimateNormals(
  small_gicp::PointCloud & cloud, KdTree & tree, int num_neighbors, int num_threads)
{
  if (cloud.empty()) {return 0;}
  const int k = std::max(5, num_neighbors);
  const int threads = std::max(1, num_threads);
  std::size_t insufficient = 0;

#pragma omp parallel num_threads(threads)
  {
    std::vector<std::size_t> indices(static_cast<std::size_t>(k));
    std::vector<double> sq_dists(static_cast<std::size_t>(k));
    std::size_t local = 0;

#pragma omp for schedule(guided, 16)
    for (std::int64_t i = 0; i < static_cast<std::int64_t>(cloud.size()); i++) {
      const std::size_t idx = static_cast<std::size_t>(i);
      const int n = static_cast<int>(tree.knn_search(
          cloud.point(idx), k, indices.data(), sq_dists.data()));
      if (n < 5) {
        cloud.normal(idx).setZero();
        local++;
        continue;
      }
      Eigen::Vector3d sum = Eigen::Vector3d::Zero();
      Eigen::Matrix3d sum_cross = Eigen::Matrix3d::Zero();
      for (int j = 0; j < n; j++) {
        const Eigen::Vector3d p = cloud.point(indices[j]).head<3>();
        sum += p;
        sum_cross += p * p.transpose();
      }
      const Eigen::Vector3d mean = sum / double(n);
      const Eigen::Matrix3d cov = (sum_cross - mean * sum.transpose()) / double(n);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
      if (es.info() != Eigen::Success) {
        cloud.normal(idx).setZero();
        local++;
        continue;
      }
      Eigen::Vector3d nrm = es.eigenvectors().col(0).normalized();
      if (cloud.point(idx).head<3>().dot(nrm) > 0) {nrm = -nrm;}
      cloud.normal(idx) << nrm.x(), nrm.y(), nrm.z(), 0.0;
    }

#pragma omp critical
    insufficient += local;
  }
  return insufficient;
}

small_gicp::PointCloud::Ptr subsetCloud(
  const small_gicp::PointCloud & cloud, const std::vector<std::size_t> & indices)
{
  auto out = std::make_shared<small_gicp::PointCloud>();
  out->resize(indices.size());
  for (std::size_t i = 0; i < indices.size(); i++) {
    const std::size_t src = indices[i];
    out->point(i) = cloud.point(src);
    out->normal(i) = cloud.normal(src);
    out->cov(i) = cloud.cov(src);
  }
  return out;
}

}  // namespace gicp15d
