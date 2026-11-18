// SPDX-License-Identifier: MIT
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <limits>

#include <small_gicp/ann/traits.hpp>
#include <small_gicp/points/traits.hpp>
#include <small_gicp/util/lie.hpp>

#include "gicp15d/types.h"

namespace gicp15d {

/// GICP point factor with an explicit IRLS robust weight and a normal-
/// compatibility gate.
///
/// The residual, the fused precision (C_target + T C_source T^T)^-1 and the
/// Gauss-Newton blocks are exactly upstream `small_gicp::GICPFactor`. On top of
/// that, every inlier's H/b/e is scaled by a robust weight w(r), where r is the
/// Euclidean norm of the point residual. This is the FAR-LIO arrangement (the
/// kernel lives inside the factor, not outside the solver) but with the kernel
/// scale expressed in metres rather than in sqrt(2 e) units, which is what
/// upstream's unused RobustFactor gets wrong.
///
/// The Hessian is the Gauss-Newton Hessian of the frozen-Mahalanobis cost, not
/// of the true cost: dM/dT is dropped. It is a geometry diagnostic only.
struct RobustFactorSetting
{
  RobustKernel kernel = RobustKernel::NONE;
  double scale_m = 1.0;
  /// cos of the maximum allowed angle between the target normal and the
  /// transformed source normal. -1 disables the gate.
  double min_normal_dot = -1.0;
};

struct RobustGICPFactor
{
  using Setting = RobustFactorSetting;

  explicit RobustGICPFactor(const RobustFactorSetting & setting = RobustFactorSetting())
  : target_index(std::numeric_limits<std::size_t>::max()),
    source_index(std::numeric_limits<std::size_t>::max()),
    setting_(setting),
    mahalanobis(Eigen::Matrix4d::Zero()),
    residual_norm(0.0),
    maha_norm(0.0),
    weight(0.0) {}

  template<typename TargetPointCloud, typename SourcePointCloud, typename TargetTree,
    typename CorrespondenceRejector>
  bool linearize(
    const TargetPointCloud & target, const SourcePointCloud & source,
    const TargetTree & target_tree, const Eigen::Isometry3d & T, std::size_t src_index,
    const CorrespondenceRejector & rejector, Eigen::Matrix<double, 6, 6> * H,
    Eigen::Matrix<double, 6, 1> * b, double * e)
  {
    source_index = src_index;
    target_index = std::numeric_limits<std::size_t>::max();
    residual_norm = 0.0;
    maha_norm = 0.0;
    weight = 0.0;

    const Eigen::Vector4d transed = T * small_gicp::traits::point(source, src_index);

    std::size_t k_index = 0;
    double k_sq_dist = 0.0;
    if (!small_gicp::traits::nearest_neighbor_search(
        target_tree, transed, &k_index, &k_sq_dist) ||
      rejector(target, source, T, k_index, src_index, k_sq_dist))
    {
      return false;
    }

    // Normal compatibility: reject pairs whose surfaces cannot be the same
    // physical surface. Evaluated in the map frame.
    if (setting_.min_normal_dot > -1.0) {
      const Eigen::Vector3d n_t = small_gicp::traits::normal(target, k_index).template head<3>();
      const Eigen::Vector3d n_s = small_gicp::traits::normal(source, src_index).template head<3>();
      if (n_t.squaredNorm() > 0.25 && n_s.squaredNorm() > 0.25) {
        const double dot = n_t.dot(T.linear() * n_s);
        if (dot < setting_.min_normal_dot) {return false;}
      }
    }

    target_index = k_index;

    const Eigen::Matrix4d RCR = small_gicp::traits::cov(target, target_index) +
      T.matrix() * small_gicp::traits::cov(source, source_index) * T.matrix().transpose();
    mahalanobis.setZero();
    mahalanobis.block<3, 3>(0, 0) = RCR.block<3, 3>(0, 0).inverse();

    const Eigen::Vector4d residual = small_gicp::traits::point(target, target_index) - transed;

    Eigen::Matrix<double, 4, 6> J = Eigen::Matrix<double, 4, 6>::Zero();
    J.block<3, 3>(0, 0) = T.linear() *
      small_gicp::skew(small_gicp::traits::point(source, source_index).template head<3>());
    J.block<3, 3>(0, 3) = -T.linear();

    *H = J.transpose() * mahalanobis * J;
    *b = J.transpose() * mahalanobis * residual;
    *e = 0.5 * residual.transpose() * mahalanobis * residual;

    residual_norm = residual.head<3>().norm();
    const double quad = residual.transpose() * mahalanobis * residual;
    maha_norm = std::sqrt(std::max(0.0, quad));
    weight = robustWeight(setting_.kernel, setting_.scale_m, residual_norm);
    if (weight < 1.0) {
      *H *= weight;
      *b *= weight;
      *e *= weight;
    }
    return true;
  }

  template<typename TargetPointCloud, typename SourcePointCloud>
  double error(
    const TargetPointCloud & target, const SourcePointCloud & source,
    const Eigen::Isometry3d & T) const
  {
    if (target_index == std::numeric_limits<std::size_t>::max()) {return 0.0;}
    const Eigen::Vector4d residual =
      small_gicp::traits::point(target, target_index) -
      T * small_gicp::traits::point(source, source_index);
    return 0.5 * weight * residual.transpose() * mahalanobis * residual;
  }

  bool inlier() const {return target_index != std::numeric_limits<std::size_t>::max();}

  /// IRLS weight for a Euclidean residual norm, in [0, 1].
  static double robustWeight(RobustKernel kernel, double scale, double r)
  {
    if (kernel == RobustKernel::NONE || !(scale > 0.0)) {return 1.0;}
    const double a = std::abs(r);
    switch (kernel) {
      case RobustKernel::HUBER:
        return a <= scale ? 1.0 : scale / a;
      case RobustKernel::GEMAN_MCCLURE: {
          const double s2 = scale * scale;
          const double d = s2 + a * a;
          return (s2 * s2) / (d * d);
        }
      case RobustKernel::NONE:
      default:
        return 1.0;
    }
  }

  std::size_t target_index;
  std::size_t source_index;
  Eigen::Matrix4d mahalanobis;
  double residual_norm;  ///< Euclidean |p_target - T p_source|, metres.
  double maha_norm;      ///< sqrt(r^T M r), the GICP whitened residual.
  double weight;         ///< Applied IRLS weight.

private:
  RobustFactorSetting setting_;
};

}  // namespace gicp15d
