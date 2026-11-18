// SPDX-License-Identifier: MIT
// Phase GICP15D research module. Independent of production a2rl_localization.
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace gicp15d {

using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

/// Registration active region, expressed in the PCD-map frame.
/// Mirrors the production cylinder contract: horizontal radius plus a Z band.
struct ActiveRegion {
  bool enabled = false;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  double radius = 120.0;
  double low_z = -1e9;
  double high_z = 1e9;
  std::uint64_t generation = 0;
};

/// How local point covariances are produced.
enum class CovarianceMode {
  /// Byte-for-byte the upstream small_gicp estimator: eigenvectors kept,
  /// eigenvalues replaced by (1e-3, 1, 1). This is the G0 baseline.
  SMALL_GICP_UNIT = 0,
  /// Keep the measured eigen-spectrum but floor each eigenvalue at
  /// floor_ratio * lambda_max. Avoids collapsing a genuinely 3-D
  /// neighbourhood into a unit-scale planar disc.
  SPECTRUM_FLOOR = 1,
  /// FAR-LIO "FROBENIUS" regularisation (config/far-lio.yml:160, default SVD):
  ///   C = cov + lambda*I,  C_far = (C^-1 / ||C^-1||_F)^-1,  lambda = 1e-3
  /// See lib_cpp/odometry_cpp/modules/map_handler/include/map_handler/
  /// map_handler_base.hpp:393-401 (CPU) and map_utils.cuh:87-95 (GPU, same math).
  FAR_FROBENIUS = 2,
};

/// Which cloud the covariances are estimated on.
enum class CovarianceDomain {
  /// Estimate once on the full static map, then crop to the ROI.
  /// Nearest-neighbour search during registration is restricted to the ROI,
  /// so a rejected candidate is never "stuck" behind an out-of-ROI neighbour.
  FULL_MAP = 0,
  /// Same, but the covariance neighbourhoods are computed after cropping.
  CROPPED_MAP = 1,
};

enum class RobustKernel {
  NONE = 0,
  HUBER = 1,
  GEMAN_MCCLURE = 2,
};

struct FrontendConfig {
  /// Human-readable preset identity recorded in every result.
  std::string preset = "G0_AW_EQUIV";

  // ---- covariance ----
  int covariance_knn = 20;
  CovarianceMode covariance_mode = CovarianceMode::SMALL_GICP_UNIT;
  CovarianceDomain covariance_domain = CovarianceDomain::FULL_MAP;
  double covariance_floor_ratio = 1.0e-3;

  // ---- association ----
  double max_correspondence_distance = 4.0;
  /// Restrict nearest-neighbour search to the ROI-cropped target.
  /// false reproduces the AW/production "global NN then ROI reject" behaviour.
  bool crop_target_to_roi = false;
  /// Reject a pair when the transformed source normal and the target normal
  /// disagree by more than this angle. 180 deg disables the check.
  double normal_compatibility_deg = 180.0;
  /// Require the target point's nearest source neighbour (in the source frame)
  /// to be the same source index. Doubles the NN cost; measured.
  bool reciprocal_check = false;

  // ---- robust loss (IRLS on the Mahalanobis residual) ----
  RobustKernel robust_kernel = RobustKernel::NONE;
  /// Kernel scale in units of sqrt(2 * e) = Mahalanobis residual norm.
  double robust_scale_m = 1.0;
  /// Re-estimate the kernel scale from the median inlier residual each pass.
  bool robust_scale_adaptive = false;

  // ---- optimizer ----
  int max_iterations = 50;
  double translation_epsilon = 1.0e-3;
  double rotation_epsilon = 1.7453292519943296e-3;
  int num_threads = 4;

  // ---- optional coarse-to-fine ----
  bool coarse_to_fine = false;
  double coarse_voxel = 1.5;
  double coarse_max_correspondence_distance = 8.0;
  int coarse_max_iterations = 30;

  // ---- multi-seed probe (offline only) ----
  std::vector<double> probe_offsets_m;  // along seed body x and y, plus 0
  bool probe_enabled = false;
};

struct RegistrationRequest {
  std::string case_id;
  std::string source_path;
  std::string target_path;
  /// Expected sha256 of the target file; empty disables verification.
  std::string expected_target_sha256;
  std::string expected_source_sha256;
  Eigen::Matrix4d seed = Eigen::Matrix4d::Identity();
  ActiveRegion region;
  std::uint64_t frame_id = 0;
  /// Optional grouping tag (corpus name, window, ordinal).
  std::string corpus;
  double offset_s = kNaN;
};

/// Per-correspondence population statistics, computed at the FINAL pose with a
/// single fresh linearisation (never reused from an optimizer iteration).
struct ResidualStatistics {
  std::size_t pairs = 0;
  std::size_t candidate_pairs = 0;
  std::size_t rejected_distance = 0;
  std::size_t rejected_normal = 0;
  std::size_t rejected_reciprocal = 0;
  std::size_t rejected_region = 0;

  double euclid_mean = kNaN, euclid_median = kNaN, euclid_p95 = kNaN, euclid_max = kNaN;
  double maha_mean = kNaN, maha_median = kNaN, maha_p95 = kNaN, maha_max = kNaN;
  double weight_mean = kNaN, weight_min = kNaN;

  /// Inlier azimuth coverage in the seed body frame: 12 sectors of 30 deg.
  std::vector<int> sector_counts;
  int sectors_occupied = 0;
  /// Fraction of inliers in the largest single sector.
  double dominant_sector_fraction = kNaN;
  /// Normalised entropy of the sector histogram (1 = uniform).
  double sector_entropy = kNaN;
  /// Inlier range distribution (metres from the seed origin).
  double range_median = kNaN, range_p95 = kNaN;
  /// Fraction of inliers whose range is below 20 m (near-field support).
  double near_field_fraction = kNaN;
};

/// Geometry-only information diagnostics. This is NOT an observation
/// covariance and must never be used as an EKF R.
struct InformationDiagnostics {
  Matrix6d H = Matrix6d::Constant(kNaN);
  Vector6d b = Vector6d::Constant(kNaN);
  double cost = kNaN;
  /// Eigen-decomposition of the symmetrised H in the right (body) chart,
  /// ordering [rotation(3), translation(3)] exactly as small_gicp's delta.
  Vector6d eigenvalues = Vector6d::Constant(kNaN);
  Matrix6d eigenvectors = Matrix6d::Constant(kNaN);
  double condition_number = kNaN;
  /// Unit translation direction with the smallest information.
  Eigen::Vector3d weakest_translation_direction = Eigen::Vector3d::Constant(kNaN);
  double weakest_translation_ratio = kNaN;
  bool rank_deficient = false;
  std::string covariance_status = "NOT_A_COVARIANCE";
};

struct TimingBreakdown {
  double target_load_ms = kNaN;
  double target_covariance_ms = kNaN;
  double target_roi_ms = kNaN;
  double target_tree_ms = kNaN;
  double source_load_ms = kNaN;
  double source_covariance_ms = kNaN;
  double source_tree_ms = kNaN;
  double align_ms = kNaN;
  double diagnostics_ms = kNaN;
  double total_ms = kNaN;
  bool target_cache_hit = false;
};

struct ProbeResult {
  double offset_m = 0.0;
  Eigen::Vector3d axis = Eigen::Vector3d::Zero();
  Eigen::Matrix4d final = Eigen::Matrix4d::Identity();
  bool converged = false;
  bool candidate_valid = false;
  std::size_t correspondences = 0;
  double cost = kNaN;
  double shift_from_seed_m = kNaN;
};

struct RegistrationOutput {
  // ---- identity ----
  std::string case_id;
  std::string preset;
  std::string backend = "small_gicp";
  std::string corpus;
  double offset_s = kNaN;
  std::string source_path;
  std::string target_path;
  std::string source_sha256;
  std::string target_sha256;
  std::size_t source_points = 0;
  std::size_t target_points = 0;
  std::size_t target_roi_points = 0;

  // ---- seed and result ----
  Eigen::Matrix4d seed = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d final = Eigen::Matrix4d::Identity();
  bool converged = false;
  bool candidate_valid = false;
  bool success = false;
  int iterations = 0;
  std::size_t inliers = 0;
  std::string exit_reason = "NONE";
  std::string backend_error;

  // ---- corrections relative to the frozen seed ----
  Eigen::Vector3d seed_translation_correction = Eigen::Vector3d::Constant(kNaN);
  double seed_yaw_correction = kNaN;
  double seed_translation_norm = kNaN;
  double seed_rotation_angle = kNaN;

  ResidualStatistics residuals;
  InformationDiagnostics information;
  TimingBreakdown timing;
  std::vector<ProbeResult> probes;

  /// Configuration actually used, serialised for the run ledger.
  std::string config_json;
};

}  // namespace gicp15d
