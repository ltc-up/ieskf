// SPDX-License-Identifier: MIT
// CPU-only contract tests for the Phase G1 frontend. No GPU, no ROS, no network.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <random>
#include <vector>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <small_gicp/ann/kdtree_omp.hpp>

#include "gicp15d/correspondence_rejector.h"
#include "gicp15d/gicp_frontend.h"
#include "gicp15d/local_covariance.h"
#include "gicp15d/point_cloud_io.h"
#include "gicp15d/registration_diagnostics.h"
#include "gicp15d/robust_gicp_factor.h"

namespace {

int g_failures = 0;
int g_checks = 0;

void check(bool ok, const std::string & what)
{
  g_checks++;
  if (!ok) {
    g_failures++;
    std::cerr << "FAIL: " << what << "\n";
  }
}

void checkNear(double a, double b, double tol, const std::string & what)
{
  g_checks++;
  if (!(std::abs(a - b) <= tol)) {
    g_failures++;
    std::cerr << "FAIL: " << what << " (" << a << " vs " << b << ", tol " << tol << ")\n";
  }
}

// ---------------------------------------------------------------- synthetic map
gicp15d::CloudPtr makeCorridor()
{
  auto c = std::make_shared<gicp15d::Cloud>();
  std::vector<Eigen::Vector3d> pts;
  for (double x = -30.0; x <= 30.0; x += 0.5) {
    for (double y = -7.0; y <= 7.0; y += 0.5) {pts.emplace_back(x, y, 0.0);}
    for (double z = 0.5; z <= 4.0; z += 0.5) {
      pts.emplace_back(x, -7.0, z);
      pts.emplace_back(x, 7.0, z);
    }
  }
  // A few asymmetric pillars so the problem is not translation-invariant.
  for (double z = 0.5; z <= 3.0; z += 0.5) {
    pts.emplace_back(4.0, 2.0, z);
    pts.emplace_back(-9.0, -3.0, z);
    pts.emplace_back(15.0, 4.0, z);
  }
  // A slanted barrier, so no integer or half-integer shift of the regular grid
  // maps the scene onto itself. Without this the corridor is genuinely
  // ambiguous along x and a 0.5 m slide is an equally valid GICP minimum.
  for (double y = -6.0; y <= 6.0; y += 0.25) {
    for (double z = 0.5; z <= 2.5; z += 0.5) {
      pts.emplace_back(22.0 + 0.35 * y + 0.2 * z, y, z);
    }
  }
  // Deterministic jitter removes the exact translational symmetry of the grid
  // while keeping the fixture reproducible across runs.
  std::mt19937 rng(20261118u);
  std::uniform_real_distribution<double> jitter(-0.13, 0.13);
  for (auto & p : pts) {p += Eigen::Vector3d(jitter(rng), jitter(rng), jitter(rng));}
  c->resize(pts.size());
  for (std::size_t i = 0; i < pts.size(); i++) {
    c->point(i) << pts[i].x(), pts[i].y(), pts[i].z(), 1.0;
  }
  return c;
}

void writeBin(const std::string & path, const gicp15d::Cloud & cloud)
{
  std::ofstream f(path, std::ios::binary);
  const std::uint64_t n = cloud.size();
  f.write(reinterpret_cast<const char *>(&n), sizeof(n));
  for (std::size_t i = 0; i < cloud.size(); i++) {
    const float xyz[3] = {
      static_cast<float>(cloud.point(i).x()),
      static_cast<float>(cloud.point(i).y()),
      static_cast<float>(cloud.point(i).z())};
    f.write(reinterpret_cast<const char *>(xyz), sizeof(xyz));
  }
}

// ---------------------------------------------------------------------- tests
void testCloudRoundTrip()
{
  auto c = makeCorridor();
  const std::string path = "/tmp/gicp15d_test_target.bin";
  writeBin(path, *c);
  std::string err;
  auto back = gicp15d::loadXyzBin(path, &err);
  check(back != nullptr, "round-trip load succeeds: " + err);
  if (!back) {return;}
  check(back->size() == c->size(), "round-trip point count preserved");
  check(std::abs(back->point(7).x() - c->point(7).x()) < 1e-6,
    "round-trip coordinate preserved within float32");
  const std::string sha = gicp15d::fileSha256(path);
  check(sha.size() == 64, "sha256 is 64 hex characters");
  check(gicp15d::loadXyzBin("/tmp/does_not_exist_15d.bin", &err) == nullptr,
    "missing file returns nullptr");
  std::printf("[ok] cloud round trip (sha %s)\n", sha.substr(0, 12).c_str());
}

void testCovarianceModes()
{
  auto c = makeCorridor();
  auto tree = std::make_shared<gicp15d::KdTree>(c, small_gicp::KdTreeBuilderOMP(1));

  auto unit = std::make_shared<gicp15d::Cloud>(*c);
  auto unit_stats = gicp15d::estimateCovariances(
    *unit, *tree, 20, gicp15d::CovarianceMode::SMALL_GICP_UNIT, 1e-3, 1);
  check(unit_stats.insufficient_neighbours == 0, "no insufficient neighbourhoods on the corridor");
  checkNear(unit_stats.lambda_max_median, 1.0, 1e-9, "unit mode lambda_max == 1");
  checkNear(unit_stats.lambda_min_median, 1e-3, 1e-12, "unit mode lambda_min == 1e-3");

  // Every unit-mode covariance must have that exact spectrum.
  bool spectrum_ok = true;
  for (std::size_t i = 0; i < unit->size(); i += 97) {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(unit->cov(i).block<3, 3>(0, 0));
    if (std::abs(es.eigenvalues()(0) - 1e-3) > 1e-9 ||
      std::abs(es.eigenvalues()(1) - 1.0) > 1e-9)
    {
      spectrum_ok = false;
      break;
    }
  }
  check(spectrum_ok, "unit mode spectrum is exactly (1e-3, 1, 1)");

  auto floor_cloud = std::make_shared<gicp15d::Cloud>(*c);
  auto floor_stats = gicp15d::estimateCovariances(
    *floor_cloud, *tree, 20, gicp15d::CovarianceMode::SPECTRUM_FLOOR, 1e-3, 1);
  check(floor_stats.condition_median > 1.0,
    "spectrum mode condition number exceeds 1 (scale retained)");
  check(std::isfinite(floor_stats.effective_floor_median), "spectrum floor median finite");

  // Floor honoured for every point.
  bool floor_ok = true;
  for (std::size_t i = 0; i < floor_cloud->size(); i++) {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(
      floor_cloud->cov(i).block<3, 3>(0, 0));
    const Eigen::Vector3d l = es.eigenvalues();
    if (l(0) < 1e-3 * l(2) * (1.0 - 1e-9)) {
      floor_ok = false;
      break;
    }
  }
  check(floor_ok, "spectrum mode never falls below floor_ratio * lambda_max");
  std::printf("[ok] covariance modes (unit cond %.1f, floor cond %.1f)\n",
    unit_stats.condition_median, floor_stats.condition_median);
}

void testRejector()
{
  auto c = makeCorridor();
  auto tree = std::make_shared<gicp15d::KdTree>(c, small_gicp::KdTreeBuilderOMP(1));
  gicp15d::estimateCovariances(*c, *tree, 20, gicp15d::CovarianceMode::SMALL_GICP_UNIT, 1e-3, 1);

  gicp15d::GicpRejector rej;
  rej.max_dist_sq = 4.0;  // 2 m gate
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  // Distance gate: 1 m apart passes, 3 m apart fails.
  check(!rej(*c, *c, T, 0, 0, 1.0), "1 m squared distance inside a 2 m gate is accepted");
  check(rej(*c, *c, T, 0, 0, 9.0), "3 m squared distance outside a 2 m gate is rejected");
  check(rej.counters.rejected_distance == 1, "distance rejection counted");

  // Region gate.
  gicp15d::GicpRejector regional;
  regional.max_dist_sq = 1e9;
  regional.region.enabled = true;
  regional.region.center = Eigen::Vector3d::Zero();
  regional.region.radius = 5.0;
  regional.region.low_z = -10.0;
  regional.region.high_z = 10.0;
  // Find a point far outside the region.
  std::size_t far = 0;
  for (std::size_t i = 0; i < c->size(); i++) {
    if (c->point(i).head<2>().norm() > 20.0) {far = i; break;}
  }
  check(regional(*c, *c, T, far, far, 0.0), "point outside the active region is rejected");
  std::size_t inside = 0;
  for (std::size_t i = 0; i < c->size(); i++) {
    if (c->point(i).head<2>().norm() < 4.0 && c->point(i).z() > -5.0 &&
      c->point(i).z() < 5.0)
    {
      inside = i;
      break;
    }
  }
  check(!regional(*c, *c, T, inside, inside, 0.0), "in-region candidate is accepted");
  std::printf("[ok] rejector distance and region semantics\n");
}

void testRobustWeights()
{
  using K = gicp15d::RobustKernel;
  checkNear(gicp15d::RobustGICPFactor::robustWeight(K::NONE, 1.0, 100.0), 1.0, 1e-12,
    "no kernel means unit weight");
  checkNear(gicp15d::RobustGICPFactor::robustWeight(K::HUBER, 1.0, 0.5), 1.0, 1e-12,
    "Huber is unity inside the scale");
  checkNear(gicp15d::RobustGICPFactor::robustWeight(K::HUBER, 1.0, 2.0), 0.5, 1e-12,
    "Huber falls as scale/r outside");
  const double gm0 = gicp15d::RobustGICPFactor::robustWeight(K::GEMAN_MCCLURE, 1.0, 0.0);
  const double gm1 = gicp15d::RobustGICPFactor::robustWeight(K::GEMAN_MCCLURE, 1.0, 1.0);
  const double gm3 = gicp15d::RobustGICPFactor::robustWeight(K::GEMAN_MCCLURE, 1.0, 3.0);
  checkNear(gm0, 1.0, 1e-12, "Geman-McClure is unity at zero residual");
  checkNear(gm1, 0.25, 1e-12, "Geman-McClure is 0.25 at r == scale");
  check(gm3 < gm1 && gm1 < gm0, "Geman-McClure decreases monotonically");
  std::printf("[ok] robust kernel weights\n");
}

void testInformationAnalysis()
{
  Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
  H.diagonal() << 100.0, 100.0, 100.0, 1.0, 10.0, 4.0;
  Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Ones();
  const auto info = gicp15d::analyseInformation(H, b, 2.5);
  checkNear(info.condition_number, 100.0, 1e-6, "condition number == 100");
  check(info.eigenvalues(0) <= info.eigenvalues(5), "eigenvalues ascending");
  check(info.rank_deficient == false, "full-rank diagonal H is not rank deficient");
  checkNear(info.weakest_translation_ratio, 0.1, 1e-9,
    "weakest translation ratio == 1/10");
  check(info.covariance_status == "NOT_A_COVARIANCE",
    "information is explicitly not labelled a covariance");

  Eigen::Matrix<double, 6, 6> H0 = Eigen::Matrix<double, 6, 6>::Zero();
  const auto info0 = gicp15d::analyseInformation(H0, b, 0.0);
  check(info0.rank_deficient, "zero H reported as rank deficient");
  std::printf("[ok] information diagnostics\n");
}

void testEndToEndRecovery()
{
  auto target = makeCorridor();

  // True map <- source transform.
  Eigen::Isometry3d truth = Eigen::Isometry3d::Identity();
  truth.translation() << 12.0, 3.0, 0.0;
  truth.linear() = Eigen::AngleAxisd(0.10, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  auto source = std::make_shared<gicp15d::Cloud>();
  source->resize(target->size());
  for (std::size_t i = 0; i < target->size(); i++) {
    // Source points expressed in the source frame: p_s = T^-1 p_t.
    source->point(i) = truth.inverse() * target->point(i);
  }

  const std::string tp = "/tmp/gicp15d_e2e_target.bin";
  const std::string sp = "/tmp/gicp15d_e2e_source.bin";
  writeBin(tp, *target);
  writeBin(sp, *source);

  gicp15d::FrontendConfig cfg = gicp15d::GicpFrontend::preset("G1_FULL");
  cfg.num_threads = 1;
  gicp15d::GicpFrontend frontend(cfg);

  gicp15d::RegistrationRequest req;
  req.case_id = "synthetic_recovery";
  req.source_path = sp;
  req.target_path = tp;
  // Seed perturbed by 0.35 m and 1.5 deg.
  req.seed = truth.matrix();
  req.seed.block<3, 1>(0, 3) += Eigen::Vector3d(0.25, -0.20, 0.05);
  req.seed.block<3, 3>(0, 0) =
    req.seed.block<3, 3>(0, 0) * Eigen::AngleAxisd(0.026, Eigen::Vector3d::UnitZ())
    .toRotationMatrix();
  req.region.enabled = false;

  const auto out = frontend.align(req);
  check(out.converged, "synthetic alignment converged, exit=" + out.exit_reason);
  check(out.candidate_valid, "synthetic alignment produced a rigid candidate");
  const double terr = (out.final.block<3, 1>(0, 3) - truth.translation()).norm();
  const double rerr = std::acos(std::min(1.0, std::max(-1.0,
    ((out.final.block<3, 3>(0, 0) * truth.linear().transpose()).trace() - 1.0) * 0.5)));
  check(terr < 0.02, "recovered translation within 2 cm (got " + std::to_string(terr) + ")");
  check(rerr < 0.005, "recovered rotation within 0.005 rad (got " + std::to_string(rerr) + ")");
  check(out.residuals.pairs > 100, "synthetic inlier count is substantial");
  check(out.information.condition_number > 0.0, "synthetic condition number positive");
  check(std::abs(out.seed_translation_norm) > 0.0, "seed correction recorded");

  // Determinism with one thread.
  const auto again = frontend.align(req);
  bool identical = true;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      if (again.final(i, j) != out.final(i, j)) {identical = false;}
    }
  }
  check(identical, "single-thread alignment is bitwise deterministic");
  std::printf("[ok] end-to-end recovery terr=%.4f m rerr=%.5f rad inliers=%zu\n",
    terr, rerr, out.residuals.pairs);
}

void testFailureModes()
{
  gicp15d::FrontendConfig cfg = gicp15d::GicpFrontend::preset("G0_AW_EQUIV");
  cfg.num_threads = 1;
  gicp15d::GicpFrontend frontend(cfg);

  gicp15d::RegistrationRequest req;
  req.case_id = "missing";
  req.source_path = "/tmp/gicp15d_missing_source.bin";
  req.target_path = "/tmp/gicp15d_missing_target.bin";
  const auto out = frontend.align(req);
  check(!out.success, "missing inputs never report success");
  check(out.exit_reason.rfind("TARGET_LOAD_FAILED", 0) == 0,
    "missing target produces TARGET_LOAD_FAILED");

  // Wrong target sha must be refused rather than silently used.
  req.target_path = "/tmp/gicp15d_e2e_target.bin";
  req.source_path = "/tmp/gicp15d_e2e_source.bin";
  req.expected_target_sha256 = std::string(64, '0');
  const auto out2 = frontend.align(req);
  check(out2.exit_reason == "TARGET_SHA256_MISMATCH",
    "target sha mismatch is refused");

  // Empty ROI must fail rather than fall back to the full map.
  req.expected_target_sha256.clear();
  req.region.enabled = true;
  req.region.center = Eigen::Vector3d(1e6, 1e6, 0.0);
  req.region.radius = 1.0;
  req.region.low_z = -1e9;
  req.region.high_z = 1e9;
  gicp15d::FrontendConfig roi = gicp15d::GicpFrontend::preset("G1_ROI_CROP");
  roi.num_threads = 1;
  gicp15d::GicpFrontend roi_frontend(roi);
  const auto out3 = roi_frontend.align(req);
  check(out3.exit_reason.rfind("TARGET_ROI_FAILED", 0) == 0,
    "empty ROI fails instead of silently using the full map");
  std::printf("[ok] failure modes\n");
}

void testProbeCostContract()
{
  auto target = makeCorridor();
  Eigen::Isometry3d truth = Eigen::Isometry3d::Identity();
  truth.translation() << 12.0, 3.0, 0.0;
  truth.linear() = Eigen::AngleAxisd(0.10, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  auto source = std::make_shared<gicp15d::Cloud>();
  source->resize(target->size());
  for (std::size_t i = 0; i < target->size(); i++) {
    source->point(i) = truth.inverse() * target->point(i);
  }
  const std::string tp = "/tmp/gicp15d_probe_target.bin";
  const std::string sp = "/tmp/gicp15d_probe_source.bin";
  writeBin(tp, *target);
  writeBin(sp, *source);

  gicp15d::FrontendConfig cfg = gicp15d::GicpFrontend::preset("G1_SPECTRUM");
  cfg.num_threads = 1;
  cfg.probe_enabled = true;
  cfg.probe_offsets_m = {0.0, 1.0};
  gicp15d::GicpFrontend frontend(cfg);

  gicp15d::RegistrationRequest req;
  req.case_id = "probe_contract";
  req.source_path = sp;
  req.target_path = tp;
  req.seed = truth.matrix();
  req.seed.block<3, 1>(0, 3) += Eigen::Vector3d(0.20, -0.15, 0.03);
  req.region.enabled = false;

  const auto out = frontend.align(req);
  check(out.candidate_valid, "probe contract: main result valid");
  check(std::isfinite(out.information.cost), "probe contract: main cost is finite");
  check(!out.probes.empty(), "probe contract: probes were produced");

  bool all_finite = true;
  bool reason_when_missing = true;
  for (const auto & p : out.probes) {
    if (!std::isfinite(p.cost)) {
      all_finite = false;
      reason_when_missing = reason_when_missing && p.correspondences == 0;
    }
  }
  check(all_finite, "probe contract: every probe cost is finite");
  check(reason_when_missing, "probe contract: a missing cost comes with zero correspondences");

  // Offset 0 must reproduce the main solution, so it must carry the same cost
  // under the same objective. This is the check that costs from different
  // candidates are actually comparable.
  bool matched = false;
  for (const auto & p : out.probes) {
    if (p.offset_m == 0.0 && p.axis.x() == 1.0) {
      const double d = std::abs(p.cost - out.information.cost);
      check(d <= 1e-9 * std::max(1.0, std::abs(out.information.cost)),
        "probe contract: zero-offset probe cost equals main cost (delta " +
        std::to_string(d) + ")");
      matched = true;
    }
  }
  check(matched, "probe contract: zero-offset probe present");
  std::printf("[ok] probe cost contract (main %.6f, probes %zu)\n",
    out.information.cost, out.probes.size());
}

}  // namespace

int main()
{
  testCloudRoundTrip();
  testCovarianceModes();
  testRejector();
  testRobustWeights();
  testInformationAnalysis();
  testEndToEndRecovery();
  testFailureModes();
  testProbeCostContract();

  std::printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
