// SPDX-License-Identifier: MIT
// Streaming small_gicp G0 matcher for the 15D closed loop.
//
// Speaks the same newline-JSON request/reply protocol as the archived CUDA
// matcher service (data/phase_ar_loop44_map0105/runtime/match_service.cpp) so
// that the frozen Phase AR/AW closed-loop driver can drive it unchanged.
//
// Request keys used: case_id, header, group, scene, source_path, target_path,
//   target_sha, roi, seed_common, q, max_iterations
// Reply keys: the subset the driver reads, plus diagnostics.
//
// The matcher is the plain G0 configuration: small_gicp point covariance
// (eigenvalues 1e-3,1,1), kNN 20, distance gate 4 m, LM 50 iterations,
// 1 mm / 0.1 deg, right perturbation. No FAR_FROBENIUS, no robust kernel,
// no normal gate, no reciprocal check, no coarse-to-fine.
#include <chrono>
#include <iostream>
#include <string>

#include <nlohmann/json.hpp>

#include "gicp15d/gicp_frontend.h"
#include "gicp15d/point_cloud_io.h"

namespace {

using json = nlohmann::json;
using Clock = std::chrono::steady_clock;

json matrixJson(const Eigen::MatrixXd & m)
{
  json a = json::array();
  for (int i = 0; i < m.rows(); i++) {
    json row = json::array();
    for (int k = 0; k < m.cols(); k++) {row.push_back(m(i, k));}
    a.push_back(row);
  }
  return a;
}

}  // namespace

int main(int argc, char ** argv)
{
  int threads = 4;
  for (int i = 1; i < argc; i++) {
    const std::string a = argv[i];
    if (a == "--threads" && i + 1 < argc) {threads = std::stoi(argv[++i]);}
  }

  gicp15d::FrontendConfig cfg = gicp15d::GicpFrontend::preset("G0_AW_EQUIV");
  cfg.num_threads = threads;
  gicp15d::GicpFrontend frontend(cfg);
  // One immutable target per run; a per-request content hash would dominate the
  // frame budget and is not part of the matcher.
  frontend.setTargetRehashPerRequest(false);

  std::string line;
  while (std::getline(std::cin, line)) {
    if (line.empty()) {continue;}
    json out;
    try {
      const json j = json::parse(line);
      gicp15d::RegistrationRequest req;
      req.case_id = j.value("case_id", std::string("case"));
      req.source_path = j.at("source_path").get<std::string>();
      req.target_path = j.at("target_path").get<std::string>();
      req.expected_target_sha256 = j.value("target_sha", std::string(""));

      for (int a = 0; a < 4; a++) {
        for (int b = 0; b < 4; b++) {
          req.seed(a, b) = j.at("seed_common").at(a).at(b).get<double>();
        }
      }

      // roi = [enabled, shape, cx, cy, cz, min_x, max_x, min_y, max_y,
      //        min_z, max_z, radius, generation]  (cylinder + box contract).
      // G0 uses the cylinder exactly as the Phase AW service did.
      const auto & roi = j.at("roi");
      req.region.enabled = roi.at(0).get<bool>();
      req.region.center << roi.at(2).get<double>(), roi.at(3).get<double>(),
        roi.at(4).get<double>();
      req.region.low_z = roi.at(9).get<double>();
      req.region.high_z = roi.at(10).get<double>();
      req.region.radius = roi.at(11).get<double>();
      req.region.generation = roi.at(12).get<std::uint64_t>();
      req.corpus = "g2_closed_loop";

      const auto t0 = Clock::now();
      const gicp15d::RegistrationOutput r = frontend.align(req);
      const double wall = std::chrono::duration<double, std::milli>(
        Clock::now() - t0).count();

      out = {
        {"case_id", r.case_id},
        {"header", j.value("header", 0ull)},
        {"group", j.value("group", std::string(""))},
        {"scene", j.value("scene", std::string(""))},
        {"backend", "small_gicp"},
        {"source_n", r.source_points},
        {"final", matrixJson(r.final)},
        {"converged", r.converged},
        {"candidate_valid", r.candidate_valid},
        {"iterations", r.iterations},
        {"exit", r.exit_reason},
        {"align_wall_ms", wall},
        {"align_internal_ms", r.timing.align_ms},
        {"source_load_ms", r.timing.source_load_ms},
        {"source_covariance_ms", r.timing.source_covariance_ms},
        {"diagnostics_ms", r.timing.diagnostics_ms},
        {"target_build_ms", r.timing.target_cache_hit ? 0.0 : r.timing.total_ms},
        {"target_cache_hit", r.timing.target_cache_hit},
        {"pairs", r.inliers},
        {"candidate_pairs", r.residuals.candidate_pairs},
        {"euclid_median_m", r.residuals.euclid_median},
        {"maha_median", r.residuals.maha_median},
        {"sectors_occupied", r.residuals.sectors_occupied},
        {"condition_number", r.information.condition_number},
        {"seed_translation_m", r.seed_translation_norm},
        {"seed_rotation_rad", r.seed_rotation_angle},
        {"error", ""}};
      if (!r.backend_error.empty()) {out["error"] = r.backend_error;}
    } catch (const std::exception & e) {
      out = {{"converged", false}, {"candidate_valid", false},
        {"exit", std::string("SERVICE_EXCEPTION")}, {"error", e.what()},
        {"backend", "small_gicp"}};
    }
    std::cout << out.dump() << std::endl;
  }
  return 0;
}
