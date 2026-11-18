// SPDX-License-Identifier: MIT
// Standalone Phase G1 validation entry point. No ROS, no Python per-frame solve.
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "gicp15d/gicp_frontend.h"
#include "gicp15d/point_cloud_io.h"

namespace {

using json = nlohmann::json;

json matrixJson(const Eigen::Matrix4d & m)
{
  json a = json::array();
  for (int i = 0; i < 4; i++) {
    json row = json::array();
    for (int j = 0; j < 4; j++) {row.push_back(m(i, j));}
    a.push_back(row);
  }
  return a;
}

json vectorJson(const Eigen::Vector3d & v)
{
  return json::array({v.x(), v.y(), v.z()});
}

json matrix6Json(const Eigen::Matrix<double, 6, 6> & m)
{
  json a = json::array();
  for (int i = 0; i < 6; i++) {
    json row = json::array();
    for (int j = 0; j < 6; j++) {row.push_back(m(i, j));}
    a.push_back(row);
  }
  return a;
}

json vector6Json(const Eigen::Matrix<double, 6, 1> & v)
{
  json a = json::array();
  for (int i = 0; i < 6; i++) {a.push_back(v(i));}
  return a;
}

double nanOr(double v)
{
  return std::isfinite(v) ? v : 0.0;
}

json resultJson(const gicp15d::RegistrationOutput & o)
{
  using namespace gicp15d;
  json j;
  j["case_id"] = o.case_id;
  j["preset"] = o.preset;
  j["backend"] = o.backend;
  j["corpus"] = o.corpus;
  j["offset_s"] = o.offset_s;
  j["source_path"] = o.source_path;
  j["target_path"] = o.target_path;
  j["source_sha256"] = o.source_sha256;
  j["target_sha256"] = o.target_sha256;
  j["source_points"] = o.source_points;
  j["target_points"] = o.target_points;
  j["target_roi_points"] = o.target_roi_points;

  j["seed"] = matrixJson(o.seed);
  j["final"] = matrixJson(o.final);
  j["converged"] = o.converged;
  j["candidate_valid"] = o.candidate_valid;
  j["success"] = o.success;
  j["iterations"] = o.iterations;
  j["inliers"] = o.inliers;
  j["exit_reason"] = o.exit_reason;
  if (!o.backend_error.empty()) {j["backend_error"] = o.backend_error;}

  j["seed_translation_correction"] = vectorJson(o.seed_translation_correction);
  j["seed_translation_norm"] = o.seed_translation_norm;
  j["seed_yaw_correction"] = o.seed_yaw_correction;
  j["seed_rotation_angle"] = o.seed_rotation_angle;

  const auto & r = o.residuals;
  json rj;
  rj["pairs"] = r.pairs;
  rj["candidate_pairs"] = r.candidate_pairs;
  rj["rejected_distance"] = r.rejected_distance;
  rj["rejected_region"] = r.rejected_region;
  rj["rejected_reciprocal"] = r.rejected_reciprocal;
  rj["inlier_ratio"] = (r.candidate_pairs > 0) ?
    double(r.pairs) / double(r.candidate_pairs) : 0.0;
  rj["euclid_mean"] = r.euclid_mean;
  rj["euclid_median"] = r.euclid_median;
  rj["euclid_p95"] = r.euclid_p95;
  rj["euclid_max"] = r.euclid_max;
  rj["maha_mean"] = r.maha_mean;
  rj["maha_median"] = r.maha_median;
  rj["maha_p95"] = r.maha_p95;
  rj["maha_max"] = r.maha_max;
  rj["weight_mean"] = r.weight_mean;
  rj["weight_min"] = r.weight_min;
  rj["sector_counts"] = r.sector_counts;
  rj["sectors_occupied"] = r.sectors_occupied;
  rj["dominant_sector_fraction"] = r.dominant_sector_fraction;
  rj["sector_entropy"] = r.sector_entropy;
  rj["range_median"] = r.range_median;
  rj["range_p95"] = r.range_p95;
  rj["near_field_fraction"] = r.near_field_fraction;
  j["residuals"] = rj;

  const auto & inf = o.information;
  json ij;
  ij["H"] = matrix6Json(inf.H);
  ij["b"] = vector6Json(inf.b);
  ij["cost"] = inf.cost;
  ij["eigenvalues"] = vector6Json(inf.eigenvalues);
  ij["condition_number"] = inf.condition_number;
  ij["weakest_translation_direction"] = vectorJson(inf.weakest_translation_direction);
  ij["weakest_translation_ratio"] = inf.weakest_translation_ratio;
  ij["rank_deficient"] = inf.rank_deficient;
  ij["covariance_status"] = inf.covariance_status;
  j["information"] = ij;

  const auto & t = o.timing;
  json tj;
  tj["target_load_ms"] = t.target_load_ms;
  tj["target_covariance_ms"] = t.target_covariance_ms;
  tj["target_roi_ms"] = t.target_roi_ms;
  tj["target_tree_ms"] = t.target_tree_ms;
  tj["source_load_ms"] = t.source_load_ms;
  tj["source_covariance_ms"] = t.source_covariance_ms;
  tj["align_ms"] = t.align_ms;
  tj["diagnostics_ms"] = t.diagnostics_ms;
  tj["total_ms"] = t.total_ms;
  tj["target_cache_hit"] = t.target_cache_hit;
  j["timing"] = tj;

  json pj = json::array();
  for (const auto & p : o.probes) {
    json e;
    e["offset_m"] = p.offset_m;
    e["axis"] = vectorJson(p.axis);
    e["final"] = matrixJson(p.final);
    e["converged"] = p.converged;
    e["candidate_valid"] = p.candidate_valid;
    e["correspondences"] = p.correspondences;
    e["cost"] = p.cost;
    e["shift_from_seed_m"] = p.shift_from_seed_m;
    pj.push_back(e);
  }
  j["probes"] = pj;
  j["config"] = json::parse(o.config_json);
  return j;
}

gicp15d::RegistrationRequest requestFromJson(const json & c)
{
  gicp15d::RegistrationRequest r;
  r.case_id = c.value("case_id", std::string("unnamed"));
  r.corpus = c.value("corpus", std::string("unspecified"));
  r.offset_s = c.value("offset_s", gicp15d::kNaN);
  r.source_path = c.at("source_path").get<std::string>();
  r.target_path = c.at("target_path").get<std::string>();
  r.expected_target_sha256 = c.value("target_sha256", std::string(""));
  r.expected_source_sha256 = c.value("source_sha256", std::string(""));
  r.frame_id = c.value("frame_id", 0ull);

  const auto & s = c.at("seed");
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {r.seed(i, j) = s.at(i).at(j).get<double>();}
  }
  if (c.contains("region")) {
    const auto & g = c.at("region");
    r.region.enabled = g.value("enabled", true);
    if (g.contains("center")) {
      r.region.center << g.at("center").at(0).get<double>(),
        g.at("center").at(1).get<double>(), g.at("center").at(2).get<double>();
    }
    r.region.radius = g.value("radius", 120.0);
    r.region.low_z = g.value("low_z", -1e9);
    r.region.high_z = g.value("high_z", 1e9);
    r.region.generation = g.value("generation", 0ull);
  }
  return r;
}

std::vector<std::string> splitCommas(const std::string & s)
{
  std::vector<std::string> out;
  std::string cur;
  for (char c : s) {
    if (c == ',') {
      if (!cur.empty()) {out.push_back(cur);}
      cur.clear();
    } else {
      cur.push_back(c);
    }
  }
  if (!cur.empty()) {out.push_back(cur);}
  return out;
}

double percentile(std::vector<double> v, double p)
{
  if (v.empty()) {return 0.0;}
  std::sort(v.begin(), v.end());
  const std::size_t i = std::min(v.size() - 1,
      static_cast<std::size_t>(std::floor(p * double(v.size() - 1))));
  return v[i];
}

void usage()
{
  std::cerr <<
    "usage: gicp15d_frontend --cases FILE.jsonl --out FILE.jsonl\n"
    "                        [--presets G0_AW_EQUIV,G1_FULL] [--summary FILE.json]\n"
    "                        [--limit N] [--threads N] [--probe]\n"
    "                        [--probe-offsets 0,2,5,8,12] [--only-case ID]\n"
    "                        [--jobs 1]\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string cases_path, out_path, presets = "G0_AW_EQUIV", summary_path;
  int limit = 0, threads = 4;
  bool probe = false;
  std::string probe_offsets = "0,2,5,8,12";
  std::string only_case;

  for (int i = 1; i < argc; i++) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string {
        if (i + 1 >= argc) {throw std::runtime_error("missing value for " + a);}
        return argv[++i];
      };
    if (a == "--cases") {cases_path = next();}
    else if (a == "--out") {out_path = next();}
    else if (a == "--presets") {presets = next();}
    else if (a == "--summary") {summary_path = next();}
    else if (a == "--limit") {limit = std::stoi(next());}
    else if (a == "--threads") {threads = std::stoi(next());}
    else if (a == "--probe") {probe = true;}
    else if (a == "--probe-offsets") {probe_offsets = next();}
    else if (a == "--only-case") {only_case = next();}
    else if (a == "--help" || a == "-h") {usage(); return 0;}
    else {usage(); return 2;}
  }
  if (cases_path.empty() || out_path.empty()) {usage(); return 2;}

  std::ifstream in(cases_path);
  if (!in) {std::cerr << "cannot open cases: " << cases_path << "\n"; return 1;}
  std::vector<json> cases;
  {
    std::string line;
    while (std::getline(in, line)) {
      if (line.empty() || line[0] == '#') {continue;}
      cases.push_back(json::parse(line));
    }
  }
  if (!only_case.empty()) {
    cases.erase(std::remove_if(cases.begin(), cases.end(),
      [&](const json & c) {return c.value("case_id", std::string()) != only_case;}),
      cases.end());
  }
  if (limit > 0 && static_cast<int>(cases.size()) > limit) {cases.resize(limit);}
  if (cases.empty()) {std::cerr << "no cases\n"; return 1;}

  std::vector<std::string> preset_names = splitCommas(presets);
  std::vector<gicp15d::FrontendConfig> configs;
  for (const auto & name : preset_names) {
    gicp15d::FrontendConfig c = gicp15d::GicpFrontend::preset(name);
    c.num_threads = threads;
    if (probe) {
      c.probe_enabled = true;
      for (const auto & s : splitCommas(probe_offsets)) {c.probe_offsets_m.push_back(std::stod(s));}
    }
    configs.push_back(c);
  }

  std::ofstream out(out_path);
  if (!out) {std::cerr << "cannot write: " << out_path << "\n"; return 1;}

  std::map<std::string, std::vector<double>> align_ms, total_ms;
  std::map<std::string, int> n_total, n_converged, n_valid, n_success;

  const auto t_start = std::chrono::steady_clock::now();
  std::size_t written = 0;
  for (const auto & cfg : configs) {
    gicp15d::GicpFrontend frontend(cfg);
    for (const auto & c : cases) {
      gicp15d::RegistrationRequest req = requestFromJson(c);
      const gicp15d::RegistrationOutput o = frontend.align(req);
      out << resultJson(o).dump() << "\n";
      out.flush();
      written++;
      const std::string & k = o.preset;
      align_ms[k].push_back(o.timing.align_ms);
      total_ms[k].push_back(o.timing.total_ms);
      n_total[k]++;
      n_converged[k] += o.converged ? 1 : 0;
      n_valid[k] += o.candidate_valid ? 1 : 0;
      n_success[k] += o.success ? 1 : 0;
    }
  }
  out.close();

  const double wall_s = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - t_start).count();

  if (!summary_path.empty()) {
    json s;
    s["cases_file"] = cases_path;
    s["cases"] = cases.size();
    s["presets"] = preset_names;
    s["threads"] = threads;
    s["results_written"] = written;
    s["wall_seconds"] = wall_s;
    json per;
    for (const auto & name : preset_names) {
      json e;
      e["frames"] = n_total[name];
      e["converged"] = n_converged[name];
      e["candidate_valid"] = n_valid[name];
      e["success"] = n_success[name];
      e["align_ms_p50"] = percentile(align_ms[name], 0.50);
      e["align_ms_p95"] = percentile(align_ms[name], 0.95);
      e["align_ms_max"] = percentile(align_ms[name], 1.0);
      e["total_ms_p50"] = percentile(total_ms[name], 0.50);
      e["total_ms_p95"] = percentile(total_ms[name], 0.95);
      e["total_ms_max"] = percentile(total_ms[name], 1.0);
      per[name] = e;
    }
    s["per_preset"] = per;
    std::ofstream sf(summary_path);
    sf << std::setw(2) << s << "\n";
  }
  std::cout << "wrote " << written << " results in " << std::fixed << std::setprecision(1)
            << wall_s << " s -> " << out_path << "\n";
  return 0;
}
