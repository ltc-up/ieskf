// SPDX-License-Identifier: MIT
#include "gicp15d/registration_diagnostics.h"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <small_gicp/registration/reduction_omp.hpp>

namespace gicp15d {
namespace {

double percentile(std::vector<double> v, double p)
{
  if (v.empty()) {return std::numeric_limits<double>::quiet_NaN();}
  std::sort(v.begin(), v.end());
  const double idx = (v.size() - 1) * p;
  const std::size_t lo = static_cast<std::size_t>(std::floor(idx));
  const std::size_t hi = std::min(v.size() - 1, lo + 1);
  const double w = idx - double(lo);
  return v[lo] * (1.0 - w) + v[hi] * w;
}

double meanOf(const std::vector<double> & v)
{
  if (v.empty()) {return std::numeric_limits<double>::quiet_NaN();}
  double s = 0.0;
  for (double x : v) {s += x;}
  return s / double(v.size());
}

}  // namespace

InformationDiagnostics analyseInformation(
  const Eigen::Matrix<double, 6, 6> & H, const Eigen::Matrix<double, 6, 1> & b,
  double cost)
{
  InformationDiagnostics out;
  out.H = H;
  out.b = b;
  out.cost = cost;
  if (!H.allFinite()) {return out;}

  const Eigen::Matrix<double, 6, 6> Hs = 0.5 * (H + H.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> es(Hs);
  if (es.info() != Eigen::Success) {return out;}
  out.eigenvalues = es.eigenvalues();      // ascending
  out.eigenvectors = es.eigenvectors();
  const double lmax = out.eigenvalues(5);
  const double lmin = out.eigenvalues(0);
  out.condition_number = (lmin > 0.0 && lmax > 0.0) ? lmax / lmin :
    std::numeric_limits<double>::infinity();
  out.rank_deficient = !(lmin > 0.0) || lmin < 1.0e-9 * std::max(lmax, 1.0e-12);

  // Translation-only information block: which horizontal/vertical direction is
  // least constrained by the current correspondences?
  const Eigen::Matrix3d Htt = Hs.block<3, 3>(3, 3);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es_t(Htt);
  if (es_t.info() == Eigen::Success) {
    const Eigen::Vector3d lt = es_t.eigenvalues();
    Eigen::Vector3d dir = es_t.eigenvectors().col(0);
    if (dir.norm() > 0.0) {
      dir.normalize();
      out.weakest_translation_direction = dir;
      out.weakest_translation_ratio = (lt(2) > 0.0) ? lt(0) / lt(2) :
        std::numeric_limits<double>::infinity();
    }
  }
  out.covariance_status = "NOT_A_COVARIANCE";
  return out;
}

void computeDiagnostics(
  const small_gicp::PointCloud & target, const small_gicp::PointCloud & source,
  const KdTree & target_tree, const GicpRejector & rejector_template,
  const RobustGICPFactor::Setting & factor_setting, int num_threads,
  const Eigen::Isometry3d & T_final, ResidualStatistics * residuals,
  InformationDiagnostics * information)
{
  ResidualStatistics res;
  InformationDiagnostics info;

  if (source.empty() || target.empty()) {
    if (residuals) {*residuals = res;}
    if (information) {*information = info;}
    return;
  }

  GicpRejector rejector = rejector_template;
  rejector.counters = GicpRejector::Counters();

  std::vector<RobustGICPFactor> factors(source.size(), RobustGICPFactor(factor_setting));
  small_gicp::ParallelReductionOMP reduction;
  reduction.num_threads = std::max(1, num_threads);

  const auto linearized = reduction.linearize(
    target, source, target_tree, rejector, T_final, factors);
  const Eigen::Matrix<double, 6, 6> H = std::get<0>(linearized);
  const Eigen::Matrix<double, 6, 1> b = std::get<1>(linearized);
  const double cost = std::get<2>(linearized);

  res.candidate_pairs = rejector.counters.total;
  res.rejected_distance = rejector.counters.rejected_distance;
  res.rejected_region = rejector.counters.rejected_region;
  res.rejected_reciprocal = rejector.counters.rejected_reciprocal;

  std::vector<double> euclid, maha, range;
  std::vector<double> weights;
  std::vector<int> sectors(12, 0);
  const Eigen::Isometry3d T_inv = T_final.inverse();
  std::size_t pairs = 0;
  std::size_t near_field = 0;

  for (const auto & f : factors) {
    if (!f.inlier()) {continue;}
    pairs++;
    euclid.push_back(f.residual_norm);
    maha.push_back(f.maha_norm);
    weights.push_back(f.weight);

    // Spatial distribution of the matched target points in the body frame.
    const Eigen::Vector4d p_body = T_inv * target.point(f.target_index);
    const double az = std::atan2(p_body.y(), p_body.x());
    int sector = static_cast<int>(std::floor((az + M_PI) / (2.0 * M_PI) * 12.0));
    sector = std::min(11, std::max(0, sector));
    sectors[static_cast<std::size_t>(sector)]++;
    const double rng = p_body.head<3>().norm();
    range.push_back(rng);
    if (rng < 20.0) {near_field++;}
  }

  res.pairs = pairs;
  res.sector_counts = sectors;
  int occupied = 0;
  int dominant = 0;
  double entropy = 0.0;
  for (int c : sectors) {
    if (c > 0) {occupied++;}
    dominant = std::max(dominant, c);
  }
  res.sectors_occupied = occupied;
  if (pairs > 0) {
    res.dominant_sector_fraction = double(dominant) / double(pairs);
    for (int c : sectors) {
      if (c == 0) {continue;}
      const double p = double(c) / double(pairs);
      entropy -= p * std::log(p);
    }
    res.sector_entropy = entropy / std::log(12.0);
    res.near_field_fraction = double(near_field) / double(pairs);
  }
  res.euclid_mean = meanOf(euclid);
  res.euclid_median = percentile(euclid, 0.5);
  res.euclid_p95 = percentile(euclid, 0.95);
  res.euclid_max = percentile(euclid, 1.0);
  res.maha_mean = meanOf(maha);
  res.maha_median = percentile(maha, 0.5);
  res.maha_p95 = percentile(maha, 0.95);
  res.maha_max = percentile(maha, 1.0);
  res.range_median = percentile(range, 0.5);
  res.range_p95 = percentile(range, 0.95);
  res.weight_mean = meanOf(weights);
  res.weight_min = percentile(weights, 0.0);

  info = analyseInformation(H, b, cost);

  if (residuals) {*residuals = res;}
  if (information) {*information = info;}
}

}  // namespace gicp15d
