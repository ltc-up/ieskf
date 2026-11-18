// SPDX-License-Identifier: MIT
#include "gicp15d/gicp_frontend.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <stdexcept>

#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/factors/general_factor.hpp>
#include <small_gicp/registration/optimizer.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration.hpp>

#include "gicp15d/correspondence_rejector.h"
#include "gicp15d/local_covariance.h"
#include "gicp15d/point_cloud_io.h"
#include "gicp15d/registration_diagnostics.h"
#include "gicp15d/robust_gicp_factor.h"

namespace gicp15d {
namespace {

using Clock = std::chrono::steady_clock;
double msSince(Clock::time_point t)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - t).count();
}

using Registration = small_gicp::Registration<
  RobustGICPFactor, small_gicp::ParallelReductionOMP, small_gicp::NullFactor,
  GicpRejector, small_gicp::LevenbergMarquardtOptimizer>;

bool isRigid(const Eigen::Matrix4d & T)
{
  if (!T.allFinite()) {return false;}
  const Eigen::Vector3d t = T.block<3, 1>(0, 3);
  if (!t.allFinite()) {return false;}
  const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  if (!(R.transpose() * R).isApprox(Eigen::Matrix3d::Identity(), 1.0e-4)) {return false;}
  return std::abs(R.determinant() - 1.0) < 1.0e-4;
}

double rotationAngle(const Eigen::Matrix3d & R)
{
  const double c = std::min(1.0, std::max(-1.0, (R.trace() - 1.0) * 0.5));
  return std::acos(c);
}

std::string jsonEscape(const std::string & s)
{
  std::ostringstream o;
  for (char c : s) {
    switch (c) {
      case '"': o << "\\\""; break;
      case '\\': o << "\\\\"; break;
      case '\n': o << "\\n"; break;
      case '\t': o << "\\t"; break;
      default: o << c;
    }
  }
  return o.str();
}

}  // namespace

const char * covarianceModeName(CovarianceMode mode)
{
  switch (mode) {
    case CovarianceMode::SMALL_GICP_UNIT: return "small_gicp_unit";
    case CovarianceMode::SPECTRUM_FLOOR: return "spectrum_floor";
    case CovarianceMode::FAR_FROBENIUS: return "far_frobenius";
  }
  return "unknown";
}

const char * robustKernelName(RobustKernel kernel)
{
  switch (kernel) {
    case RobustKernel::HUBER: return "huber";
    case RobustKernel::GEMAN_MCCLURE: return "geman_mcclure";
    case RobustKernel::NONE:
    default: return "none";
  }
}

std::string frontendConfigJson(const FrontendConfig & c)
{
  auto b = [](bool v) {return v ? "true" : "false";};
  std::ostringstream o;
  o << "{";
  o << "\"preset\":\"" << jsonEscape(c.preset) << "\"";
  o << ",\"covariance_knn\":" << c.covariance_knn;
  o << ",\"covariance_mode\":\"" << covarianceModeName(c.covariance_mode) << "\"";
  o << ",\"covariance_domain\":\""
    << (c.covariance_domain == CovarianceDomain::FULL_MAP ? "full_map" : "cropped_map") << "\"";
  o << ",\"covariance_floor_ratio\":" << c.covariance_floor_ratio;
  o << ",\"max_correspondence_distance\":" << c.max_correspondence_distance;
  o << ",\"crop_target_to_roi\":" << b(c.crop_target_to_roi);
  o << ",\"normal_compatibility_deg\":" << c.normal_compatibility_deg;
  o << ",\"reciprocal_check\":" << b(c.reciprocal_check);
  o << ",\"robust_kernel\":\"" << robustKernelName(c.robust_kernel) << "\"";
  o << ",\"robust_scale_m\":" << c.robust_scale_m;
  o << ",\"robust_scale_adaptive\":" << b(c.robust_scale_adaptive);
  o << ",\"max_iterations\":" << c.max_iterations;
  o << ",\"translation_epsilon\":" << c.translation_epsilon;
  o << ",\"rotation_epsilon\":" << c.rotation_epsilon;
  o << ",\"num_threads\":" << c.num_threads;
  o << ",\"coarse_to_fine\":" << b(c.coarse_to_fine);
  o << ",\"coarse_voxel\":" << c.coarse_voxel;
  o << ",\"coarse_max_correspondence_distance\":" << c.coarse_max_correspondence_distance;
  o << ",\"coarse_max_iterations\":" << c.coarse_max_iterations;
  o << ",\"probe_enabled\":" << b(c.probe_enabled);
  o << ",\"probe_offsets_m\":[";
  for (std::size_t i = 0; i < c.probe_offsets_m.size(); i++) {
    if (i) {o << ",";}
    o << c.probe_offsets_m[i];
  }
  o << "]}";
  return o.str();
}

FrontendConfig GicpFrontend::preset(const std::string & name)
{
  FrontendConfig c;
  c.preset = name;

  if (name == "G0_AW_EQUIV") {
    // Phase AW gicp_service.cpp settings, no extra mechanism.
    c.covariance_knn = 20;
    c.covariance_mode = CovarianceMode::SMALL_GICP_UNIT;
    c.covariance_domain = CovarianceDomain::FULL_MAP;
    c.max_correspondence_distance = 4.0;
    c.crop_target_to_roi = false;
    c.normal_compatibility_deg = 180.0;
    c.reciprocal_check = false;
    c.robust_kernel = RobustKernel::NONE;
    c.max_iterations = 50;
    c.translation_epsilon = 1.0e-3;
    c.rotation_epsilon = 1.7453292519943296e-3;
    return c;
  }
  if (name == "G0_SMALL_GICP_NATIVE") {
    // Upstream library defaults with only the distance gate set: a control that
    // isolates "did we change the solver settings at all?".
    c.covariance_knn = 20;
    c.covariance_mode = CovarianceMode::SMALL_GICP_UNIT;
    c.max_correspondence_distance = 4.0;
    c.max_iterations = 20;
    return c;
  }

  // Every G1_* preset starts from the G0_AW_EQUIV solver settings so that the
  // only difference is the mechanism under test.
  FrontendConfig base = preset("G0_AW_EQUIV");
  base.preset = name;

  if (name == "G1_FULL") {
    base.covariance_mode = CovarianceMode::SPECTRUM_FLOOR;
    base.covariance_floor_ratio = 1.0e-3;
    base.crop_target_to_roi = true;
    base.normal_compatibility_deg = 60.0;
    base.reciprocal_check = true;
    base.robust_kernel = RobustKernel::GEMAN_MCCLURE;
    base.robust_scale_m = 1.0;
    base.robust_scale_adaptive = false;
    return base;
  }
  if (name == "G1_FAR_FROBENIUS") {
    base.covariance_mode = CovarianceMode::FAR_FROBENIUS;
    return base;
  }
  if (name == "G1_SPECTRUM") {
    base.covariance_mode = CovarianceMode::SPECTRUM_FLOOR;
    base.covariance_floor_ratio = 1.0e-3;
    return base;
  }
  if (name == "G1_ROI_CROP") {
    base.crop_target_to_roi = true;
    return base;
  }
  if (name == "G1_NORMAL_GATE") {
    base.normal_compatibility_deg = 60.0;
    return base;
  }
  if (name == "G1_RECIPROCAL") {
    base.reciprocal_check = true;
    return base;
  }
  if (name == "G1_ROBUST_GM") {
    base.robust_kernel = RobustKernel::GEMAN_MCCLURE;
    base.robust_scale_m = 1.0;
    return base;
  }
  if (name == "G1_ROBUST_HUBER") {
    base.robust_kernel = RobustKernel::HUBER;
    base.robust_scale_m = 1.0;
    return base;
  }
  if (name == "G1_COARSE") {
    base.coarse_to_fine = true;
    return base;
  }
  throw std::invalid_argument("unknown preset: " + name);
}

GicpFrontend::GicpFrontend(const FrontendConfig & config)
: config_(config),
  maps_(config.covariance_mode, config.covariance_knn, config.covariance_floor_ratio,
    config.num_threads, 5000000)
{
}

void GicpFrontend::clearCaches()
{
  maps_.clear();
  coarse_target_.reset();
  coarse_target_key_.clear();
  roi_target_.reset();
  roi_target_key_.clear();
}

GicpFrontend::AlignPass GicpFrontend::runPass(
  const small_gicp::PointCloud & target, const KdTree & target_tree,
  const small_gicp::PointCloud & source, const KdTree & source_tree,
  const ActiveRegion & region, const Eigen::Isometry3d & init,
  int max_iterations, double max_corr, bool use_robust) const
{
  AlignPass pass;
  Registration registration;
  registration.optimizer.max_iterations = std::max(1, max_iterations);
  registration.criteria.translation_eps = config_.translation_epsilon;
  registration.criteria.rotation_eps = config_.rotation_epsilon;
  registration.reduction.num_threads = std::max(1, config_.num_threads);

  GicpRejector rejector;
  rejector.max_dist_sq = max_corr * max_corr;
  rejector.region = region;
  rejector.reciprocal_check = config_.reciprocal_check;
  rejector.source_tree = &source_tree;

  RobustGICPFactor::Setting setting;
  setting.kernel = use_robust ? config_.robust_kernel : RobustKernel::NONE;
  setting.scale_m = config_.robust_scale_m;
  setting.min_normal_dot = (config_.normal_compatibility_deg >= 180.0) ? -1.0 :
    std::cos(config_.normal_compatibility_deg * M_PI / 180.0);

  registration.rejector = rejector;
  registration.point_factor = setting;

  const auto t0 = Clock::now();
  const small_gicp::RegistrationResult r =
    registration.align(target, source, target_tree, init);
  pass.align_ms = msSince(t0);
  pass.final = r.T_target_source;
  pass.converged = r.converged;
  pass.iterations = static_cast<int>(r.iterations);
  pass.inliers = r.num_inliers;
  pass.cost = r.error;
  pass.counters = registration.rejector.counters;
  return pass;
}

RegistrationOutput GicpFrontend::probe(
  const RegistrationRequest & request, const small_gicp::PointCloud & target,
  const KdTree & target_tree, const small_gicp::PointCloud & source,
  const KdTree & source_tree, double offset_m, const Eigen::Vector3d & axis)
{
  RegistrationOutput out;
  Eigen::Isometry3d init(request.seed);
  init.translation() += offset_m * (init.linear() * axis);
  const AlignPass pass = runPass(
    target, target_tree, source, source_tree, request.region, init,
    config_.max_iterations, config_.max_correspondence_distance, true);

  out.final = pass.final.matrix();
  out.converged = pass.converged;
  out.candidate_valid = isRigid(out.final) && pass.inliers > 0;
  out.iterations = pass.iterations;
  out.inliers = pass.inliers;
  out.seed_translation_correction =
    (pass.final.translation() - request.seed.block<3, 1>(0, 3));
  out.seed_translation_norm = out.seed_translation_correction.norm();
  out.seed_rotation_angle =
    rotationAngle(pass.final.linear() * request.seed.block<3, 3>(0, 0).transpose());
  out.exit_reason = pass.converged ? "CONVERGED" : "NOT_CONVERGED";

  // The optimizer's stored cost belongs to its last linearisation pose, and the
  // factor's Mahalanobis term is frozen there. Candidate costs are only
  // comparable if every probe is re-costed at its own returned pose with the
  // same rejector, factor settings, objective and residual definition as the
  // main result. Without this the probe cost was left as NaN.
  if (out.candidate_valid) {
    GicpRejector rejector;
    rejector.max_dist_sq = config_.max_correspondence_distance *
      config_.max_correspondence_distance;
    rejector.region = request.region;
    rejector.reciprocal_check = config_.reciprocal_check;
    rejector.source_tree = &source_tree;

    RobustGICPFactor::Setting setting;
    setting.kernel = config_.robust_kernel;
    setting.scale_m = config_.robust_scale_m;
    setting.min_normal_dot = (config_.normal_compatibility_deg >= 180.0) ? -1.0 :
      std::cos(config_.normal_compatibility_deg * M_PI / 180.0);

    computeDiagnostics(
      target, source, target_tree, rejector, setting, config_.num_threads,
      pass.final, &out.residuals, &out.information);
    if (!std::isfinite(out.information.cost)) {
      out.exit_reason = "PROBE_COST_UNAVAILABLE";
    }
  }
  return out;
}

RegistrationOutput GicpFrontend::align(const RegistrationRequest & request)
{
  const auto t_total = Clock::now();
  RegistrationOutput out;
  out.case_id = request.case_id;
  out.preset = config_.preset;
  out.corpus = request.corpus;
  out.offset_s = request.offset_s;
  out.source_path = request.source_path;
  out.target_path = request.target_path;
  out.seed = request.seed;
  out.final = request.seed;
  out.config_json = frontendConfigJson(config_);

  std::string error;

  // ---- target ----
  const auto t_target = Clock::now();
  bool target_cache_hit = false;
  maps_.setRehashOnLookup(target_rehash_);
  auto full = maps_.load(request.target_path, &error, &target_cache_hit);
  if (!full) {
    out.exit_reason = "TARGET_LOAD_FAILED:" + error;
    out.timing.total_ms = msSince(t_total);
    return out;
  }
  if (!request.expected_target_sha256.empty() &&
    full->sha256 != request.expected_target_sha256)
  {
    out.exit_reason = "TARGET_SHA256_MISMATCH";
    out.timing.total_ms = msSince(t_total);
    return out;
  }
  out.target_sha256 = full->sha256;
  out.target_points = full->points_full;
  out.timing.target_load_ms = full->load_ms;
  out.timing.target_covariance_ms = full->covariance_ms;
  out.timing.target_tree_ms = full->tree_ms;
  out.timing.target_cache_hit = target_cache_hit;

  std::shared_ptr<const TargetSet> active = full;
  const std::string want_key = full->sha256 + "#" + std::to_string(request.region.generation);
  if (config_.crop_target_to_roi && request.region.enabled) {
    out.timing.target_roi_ms = msSince(t_target);
    if (!roi_target_ || roi_target_key_ != want_key) {
      roi_target_ = maps_.crop(full, request.region, &error);
      if (!roi_target_) {
        out.exit_reason = "TARGET_ROI_FAILED:" + error;
        out.timing.total_ms = msSince(t_total);
        return out;
      }
      roi_target_key_ = want_key;
    }
    active = roi_target_;
    out.timing.target_roi_ms = msSince(t_target);
  } else {
    out.timing.target_roi_ms = 0.0;
  }
  out.target_roi_points = active->points;

  // ---- source ----
  const auto t_source = Clock::now();
  auto source = loadXyzBinChecked(
    request.source_path, static_cast<std::size_t>(std::max(5, config_.covariance_knn)),
    2000000, &error);
  if (!source) {
    out.exit_reason = "SOURCE_LOAD_FAILED:" + error;
    out.timing.total_ms = msSince(t_total);
    return out;
  }
  out.timing.source_load_ms = msSince(t_source);
  out.source_points = source->size();
  out.source_sha256 = fileSha256(request.source_path);
  if (!request.expected_source_sha256.empty() &&
    out.source_sha256 != request.expected_source_sha256)
  {
    out.exit_reason = "SOURCE_SHA256_MISMATCH";
    out.timing.total_ms = msSince(t_total);
    return out;
  }

  auto source_tree = std::make_shared<KdTree>(
    source, small_gicp::KdTreeBuilderOMP(std::max(1, config_.num_threads)));
  const auto t_cov = Clock::now();
  estimateCovariances(
    *source, *source_tree, config_.covariance_knn, config_.covariance_mode,
    config_.covariance_floor_ratio, config_.num_threads);
  out.timing.source_covariance_ms = msSince(t_cov);
  out.timing.source_tree_ms = 0.0;

  // ---- optional coarse pass ----
  Eigen::Isometry3d init(request.seed);
  if (config_.coarse_to_fine) {
    const std::string coarse_key = full->sha256 + "#" +
      std::to_string(request.region.generation) + "#" + std::to_string(config_.coarse_voxel);
    if (!coarse_target_ || coarse_target_key_ != coarse_key) {
      auto coarse_cloud = voxelDownsample(*active->cloud, config_.coarse_voxel);
      auto set = std::make_shared<TargetSet>();
      set->path = full->path;
      set->sha256 = full->sha256;
      set->cropped = active->cropped;
      set->roi_key = coarse_key;
      set->cloud = coarse_cloud;
      set->points = coarse_cloud->size();
      set->points_full = full->points_full;
      set->tree = std::make_shared<KdTree>(
        coarse_cloud, small_gicp::KdTreeBuilderOMP(std::max(1, config_.num_threads)));
      estimateCovariances(
        *coarse_cloud, *set->tree, config_.covariance_knn, config_.covariance_mode,
        config_.covariance_floor_ratio, config_.num_threads);
      coarse_target_ = set;
      coarse_target_key_ = coarse_key;
    }
    auto coarse_source = voxelDownsample(*source, config_.coarse_voxel);
    auto coarse_source_tree = std::make_shared<KdTree>(
      coarse_source, small_gicp::KdTreeBuilderOMP(std::max(1, config_.num_threads)));
    estimateCovariances(
      *coarse_source, *coarse_source_tree, config_.covariance_knn, config_.covariance_mode,
      config_.covariance_floor_ratio, config_.num_threads);
    const AlignPass coarse = runPass(
      *coarse_target_->cloud, *coarse_target_->tree, *coarse_source, *coarse_source_tree,
      request.region, init, config_.coarse_max_iterations,
      config_.coarse_max_correspondence_distance, false);
    init = coarse.final;
  }

  // ---- fine pass at the configured solver settings ----
  const AlignPass pass = runPass(
    *active->cloud, *active->tree, *source, *source_tree, request.region, init,
    config_.max_iterations, config_.max_correspondence_distance, true);

  out.final = pass.final.matrix();
  out.converged = pass.converged;
  out.iterations = pass.iterations;
  out.inliers = pass.inliers;
  out.timing.align_ms = pass.align_ms;
  out.candidate_valid = isRigid(out.final) && pass.inliers > 0;
  out.success = out.converged && out.candidate_valid;
  out.exit_reason = pass.inliers == 0 ? "NO_CORRESPONDENCES" :
    (out.success ? "CONVERGED" : "NOT_CONVERGED_OR_INVALID");

  if (out.candidate_valid) {
    out.seed_translation_correction =
      out.final.block<3, 1>(0, 3) - request.seed.block<3, 1>(0, 3);
    out.seed_translation_norm = out.seed_translation_correction.norm();
    out.seed_rotation_angle = rotationAngle(
      out.final.block<3, 3>(0, 0) * request.seed.block<3, 3>(0, 0).transpose());

    const auto t_diag = Clock::now();
    GicpRejector rejector;
    rejector.max_dist_sq = config_.max_correspondence_distance *
      config_.max_correspondence_distance;
    rejector.region = request.region;
    rejector.reciprocal_check = config_.reciprocal_check;
    rejector.source_tree = source_tree.get();

    RobustGICPFactor::Setting setting;
    setting.kernel = config_.robust_kernel;
    setting.scale_m = config_.robust_scale_m;
    setting.min_normal_dot = (config_.normal_compatibility_deg >= 180.0) ? -1.0 :
      std::cos(config_.normal_compatibility_deg * M_PI / 180.0);

    computeDiagnostics(
      *active->cloud, *source, *active->tree, rejector, setting, config_.num_threads,
      Eigen::Isometry3d(out.final), &out.residuals, &out.information);
    out.timing.diagnostics_ms = msSince(t_diag);
  } else {
    out.exit_reason = "INVALID_POSE";
  }

  // ---- multi-seed probes ----
  if (config_.probe_enabled && config_.probe_offsets_m.size() > 0) {
    for (double off : config_.probe_offsets_m) {
      for (int axis = 0; axis < 2; axis++) {
        if (off == 0.0 && axis == 1) {continue;}
        Eigen::Vector3d a = Eigen::Vector3d::Zero();
        a(axis) = 1.0;
        ProbeResult pr;
        pr.offset_m = off;
        pr.axis = a;
        const RegistrationOutput one =
          probe(request, *active->cloud, *active->tree, *source, *source_tree, off, a);
        pr.final = one.final;
        pr.converged = one.converged;
        pr.candidate_valid = one.candidate_valid;
        pr.correspondences = one.inliers;
        pr.cost = one.information.cost;
        pr.shift_from_seed_m = one.seed_translation_norm;
        out.probes.push_back(pr);
      }
    }
  }

  out.timing.total_ms = msSince(t_total);
  return out;
}

}  // namespace gicp15d
