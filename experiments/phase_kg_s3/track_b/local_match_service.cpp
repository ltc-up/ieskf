// Independent research wrapper; source implementation remains private upstream.
// Each local request owns one matcher, initialized once, destroyed before reply.
#include "ndt_gpu_matcher/ndt_gpu_matcher.hpp"
#include <nlohmann/json.hpp>
#include <Eigen/Geometry>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
using J=nlohmann::json;
using Clock=std::chrono::steady_clock;
double elapsed(Clock::time_point t){return std::chrono::duration<double,std::milli>(Clock::now()-t).count();}
ndt_gpu::PointCloud load(const std::string& name){
 std::ifstream f(name,std::ios::binary);uint64_t n=0;f.read(reinterpret_cast<char*>(&n),8);
 if(!f||n>1000000)throw std::runtime_error("cloud count invalid");
 ndt_gpu::PointCloud p(n);f.read(reinterpret_cast<char*>(p.data()),n*12);
 if(!f)throw std::runtime_error("cloud truncated");return p;
}
J matrix(const Eigen::Matrix4f& T){J a=J::array();for(int i=0;i<4;++i){J r=J::array();for(int k=0;k<4;++k)r.push_back(T(i,k));a.push_back(r);}return a;}
int main(){std::string line;while(std::getline(std::cin,line)){
 auto total=Clock::now();J out;try{
 auto q=J::parse(line);auto source=load(q.at("source_path")),target=load(q.at("target_path"));
 ndt_gpu::NdtGpuConfig c;c.resolution=q.value("resolution",1.5);
 if(c.resolution!=1.5f && !(q.value("comparison",std::string())=="B-K-MR" && c.resolution==3.0f))throw std::runtime_error("unapproved resolution");
 c.step_size=.2;c.max_iterations=35;c.transformation_epsilon=.01;c.relative_objective_epsilon=.001;c.neighbor_mode=ndt_gpu::NeighborMode::DIRECT7;c.line_search_mode=ndt_gpu::LineSearchMode::GLL;c.gll_window_size=3;c.mahalanobis_gate_enabled=true;c.mahalanobis_threshold_sq=50;c.enable_pcl_fit_score=true;c.pcl_fit_max_shell=6;
 Eigen::Matrix4f seed;for(int i=0;i<4;++i)for(int k=0;k<4;++k)seed(i,k)=q.at("initial_guess").at(i).at(k);
 auto tick=Clock::now();auto matcher=std::make_unique<ndt_gpu::NdtGpuMatcher>(c);matcher->setGlobalTarget(target,q.at("target_version"));double build=elapsed(tick);
 ndt_gpu::ActiveRegion region;region.enabled=false;matcher->setActiveRegion(region);
 auto stats=matcher->globalMapStatistics();tick=Clock::now();auto r=matcher->align(source,seed);double align=elapsed(tick);tick=Clock::now();auto quality=matcher->evaluateFinalQuality(source,r.final_transform);double quality_ms=elapsed(tick);
 out={{"O_T_base",matrix(r.final_transform)},{"converged",r.converged},{"candidate_valid",r.candidate_valid},{"exit",ndt_gpu::toString(r.exit_reason)},{"iterations",r.iterations},{"optimizer_score",r.optimizer_score},{"valid_pairs",r.optimizer_valid_pairs},{"candidate_pairs",r.optimizer_candidate_pairs},{"neighbor_points",quality.valid_neighbor_count},{"inliers",quality.inlier_count},{"source_points",source.size()},{"target_points",target.size()},{"target_valid_voxels",stats.valid_voxels},{"target_build_ms",build},{"resolution",c.resolution},{"align_wall_ms",align},{"quality_extra_ms",quality_ms},{"error",r.error}};
 tick=Clock::now();matcher.reset();out["destroy_ms"]=elapsed(tick);
 }catch(const std::exception& e){out={{"error",e.what()},{"candidate_valid",false},{"converged",false}};}
 out["service_total_ms"]=elapsed(total);std::cout<<out.dump()<<std::endl;
 }return 0;}
