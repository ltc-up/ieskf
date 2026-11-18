// Offline JSONL protocol bridge to the production lidar_ndt GICP adapter.
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <nlohmann/json.hpp>
#include "gicp15d/point_cloud_io.h"
#include "lidar_ndt/gicp_registration.h"

namespace {
using json = nlohmann::json;
using namespace a2rl_lidar_ndt;
using Clock = std::chrono::steady_clock;

RegistrationPointCloud::Ptr load(const std::string& path) {
  std::string error;
  auto xyz = gicp15d::loadXyzBin(path, &error);
  if (!xyz) throw std::runtime_error("CLOUD_LOAD_" + error);
  auto out = RegistrationPointCloud::Ptr(new RegistrationPointCloud);
  out->reserve(xyz->size());
  for (std::size_t i=0; i<xyz->size(); ++i) {
    const auto& p = xyz->point(i);
    LidarPointType q; q.x=static_cast<float>(p.x()); q.y=static_cast<float>(p.y());
    q.z=static_cast<float>(p.z()); q.intensity=0; out->push_back(q);
  }
  return out;
}

json matrix(const Eigen::Matrix4f& m) {
  json a=json::array();
  for (int i=0;i<4;++i) {json row=json::array(); for(int k=0;k<4;++k) row.push_back(double(m(i,k))); a.push_back(row);}
  return a;
}
}

int main(int argc, char** argv) {
  int threads=4;
  double roi_reuse_m=0;
  for(int i=1;i<argc;++i) {
    const std::string arg=argv[i];
    if(arg=="--threads" && i+1<argc) threads=std::stoi(argv[++i]);
    if(arg=="--roi-reuse-m" && i+1<argc) roi_reuse_m=std::stod(argv[++i]);
  }
  if(roi_reuse_m<0 || roi_reuse_m>10) throw std::invalid_argument("ROI_REUSE_EXCEEDS_SOURCE_MARGIN");
  GicpConfig cfg; cfg.num_threads=threads; cfg.max_target_points=1000000;
  GicpRegistration matcher(GicpRegistration::Backend::SMALL_GICP,cfg);
  RegistrationPointCloud::Ptr full;
  RegistrationPointCloud::Ptr cached_target;
  double cached_cx=0,cached_cy=0,cached_z0=0,cached_z1=0,cached_radius=0;
  std::string target_path;
  std::string line;
  while(std::getline(std::cin,line)) {
    if(line.empty()) continue;
    json out;
    try {
      const auto j=json::parse(line);
      const std::string path=j.at("target_path").get<std::string>();
      if(!full || path!=target_path) {
        const std::string expected=j.value("target_sha",std::string());
        if(!expected.empty() && gicp15d::fileSha256(path)!=expected)
          throw std::runtime_error("TARGET_SHA_MISMATCH");
        full=load(path);target_path=path;cached_target.reset();
      }
      const auto roi=j.at("roi");
      RegistrationPointCloud::Ptr target=full;
      if(roi.at(0).get<bool>()) {
        const double cx=roi.at(2).get<double>(),cy=roi.at(3).get<double>();
        const double z0=roi.at(9).get<double>(),z1=roi.at(10).get<double>(),r=roi.at(11).get<double>();
        const bool reuse=roi_reuse_m>0 && cached_target &&
          std::hypot(cx-cached_cx,cy-cached_cy)<roi_reuse_m &&
          std::abs(z0-cached_z0)<5.0 && std::abs(z1-cached_z1)<5.0 && r==cached_radius;
        if(reuse) target=cached_target;
        else {
          target.reset(new RegistrationPointCloud); target->reserve(full->size()/4);
          for(const auto& p:*full) {
            const double dx=p.x-cx,dy=p.y-cy;
            if(dx*dx+dy*dy<=r*r && p.z>=z0 && p.z<=z1) target->push_back(p);
          }
          cached_target=target;cached_cx=cx;cached_cy=cy;
          cached_z0=z0;cached_z1=z1;cached_radius=r;
        }
      }
      auto source=load(j.at("source_path").get<std::string>());
      RegistrationInput input; input.source=source; input.target=target;
      input.frame_id=j.value("header",std::uint64_t(0));
      input.map_version=std::uint64_t(1);
      for(int a=0;a<4;++a) for(int b=0;b<4;++b)
        input.initial_guess(a,b)=j.at("seed_common").at(a).at(b).get<float>();
      const auto start=Clock::now();
      const auto r=matcher.align(input);
      const double wall=std::chrono::duration<double,std::milli>(Clock::now()-start).count();
      out={{"case_id",j.value("case_id",std::string("case"))},
        {"header",j.value("header",std::uint64_t(0))},
        {"group",j.value("group",std::string(""))},
        {"scene",j.value("scene",std::string(""))},
        {"backend","small_gicp"},{"source_n",r.source_points},
        {"final",matrix(r.final_transform)},{"converged",r.converged},
        {"candidate_valid",r.candidate_valid},{"iterations",r.iterations},
        {"exit",r.exit_reason},{"align_wall_ms",wall},
        {"align_internal_ms",r.diagnostics.optimization_ms},
        {"source_load_ms",0.0},{"source_covariance_ms",r.diagnostics.source_covariance_ms},
        {"diagnostics_ms",0.0},{"target_build_ms",r.diagnostics.target_build_ms},
        {"target_cache_hit",r.diagnostics.target_cache_hit},
        {"pairs",r.correspondences},{"candidate_pairs",r.optimizer_candidate_pairs},
        {"fitness",r.match_score},{"error",r.backend_error}};
    } catch(const std::exception& e) {
      out={{"converged",false},{"candidate_valid",false},{"exit","SERVICE_EXCEPTION"},
        {"error",e.what()},{"backend","small_gicp"}};
    }
    std::cout<<out.dump()<<std::endl;
  }
}
