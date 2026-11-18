// KG-S3 independent protocol adapter. Restricted matcher implementation is
// separately supplied under ignored data/private; no map/source is embedded.
#include "ndt_gpu_matcher/ndt_gpu_matcher.hpp"
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <tuple>
using J=nlohmann::json;
using namespace ndt_gpu;
using Clock=std::chrono::steady_clock;
static PointCloud readCloud(const std::string& path) {
 std::ifstream stream(path,std::ios::binary);uint64_t n=0;
 stream.read(reinterpret_cast<char*>(&n),8);
 if(!stream || n>100000000)throw std::runtime_error("INVALID_CLOUD_HEADER");
 PointCloud points(n);stream.read(reinterpret_cast<char*>(points.data()),n*sizeof(PointXYZ));
 if(!stream)throw std::runtime_error("INCOMPLETE_CLOUD");return points;
}
static J matrix(const Eigen::MatrixXd& m) {
 J rows=J::array();for(int i=0;i<m.rows();++i){J row=J::array();for(int j=0;j<m.cols();++j)row.push_back(m(i,j));rows.push_back(row);}return rows;
}
static J scoreJson(const ResearchScore& s) {
 return J{{"score",s.score},{"gradient",matrix(s.gradient.transpose())[0]},
 {"hessian",matrix(s.hessian)},{"valid_pairs",s.valid_pairs}};
}
int main() {
 try {
  NdtGpuConfig c;c.resolution=1.5;c.step_size=.2;c.max_iterations=35;
  c.transformation_epsilon=.01;c.relative_objective_epsilon=.001;
  c.neighbor_mode=NeighborMode::DIRECT7;c.neighbor_radius=1.5;
  c.mahalanobis_gate_enabled=true;c.mahalanobis_threshold_sq=50;
  c.optimizer_mode=OptimizerMode::MORE_THUENTE;c.line_search_mode=LineSearchMode::GLL;
  c.gll_window_size=3;c.enable_pcl_fit_score=true;c.pcl_fit_max_shell=6;
  NdtGpuMatcher matcher(c);std::string target_identity;std::string line;
  while(std::getline(std::cin,line)) {
   auto begin=Clock::now();J req=J::parse(line);
   std::string identity=req.at("target_sha");
   if(identity!=target_identity){matcher.setGlobalTarget(readCloud(req.at("target_path")),1);target_identity=identity;}
   auto r=req.at("roi");ActiveRegion ar;ar.enabled=r[0];ar.shape=static_cast<ActiveRegionShape>(r[1].get<int>());
   ar.center_x=r[2];ar.center_y=r[3];ar.center_z=r[4];ar.min_x=r[5];ar.max_x=r[6];ar.min_y=r[7];ar.max_y=r[8];ar.min_z=r[9];ar.max_z=r[10];ar.radius=r[11];ar.generation=r[12];matcher.setActiveRegion(ar);
   PointCloud source=readCloud(req.at("source_path"));J result;
   if(req.value("op",std::string("align"))=="align") {
    Eigen::Matrix4f seed;for(int i=0;i<4;++i)for(int k=0;k<4;++k)seed(i,k)=req.at("seed_common")[i][k];
    auto raw=matcher.align(source,seed);
    result={{"final",matrix(raw.final_transform.cast<double>())},{"fitness",raw.pcl_fit_score},
     {"converged",raw.converged},{"candidate_valid",raw.candidate_valid},{"iterations",raw.iterations},
     {"exit",toString(raw.exit_reason)},{"optimizer_score",raw.optimizer_score},
     {"score_gradient",matrix(raw.final_score_gradient.transpose())[0]},
     {"score_hessian",matrix(raw.final_score_hessian)},
     {"valid_pairs",raw.optimizer_valid_pairs},{"error",raw.error}};
   } else {
    Eigen::Matrix<double,6,1> q;for(int i=0;i<6;++i)q[i]=req.at("pose_q")[i];
    auto original=matcher.evaluateResearchScore(source,q,false);
    auto corrected=matcher.evaluateResearchScore(source,q,true);
    result=scoreJson(req.value("corrected_pitch",true)?corrected:original);
    result["original"]=scoreJson(original);result["corrected"]=scoreJson(corrected);
    if(req.value("spatial_blocks",false)) {
     std::map<std::tuple<int,int,int>,PointCloud> blocks;
     for(const auto& p:source)blocks[{int(std::floor(p.x/8)),int(std::floor(p.y/8)),int(std::floor(p.z/8))}].push_back(p);
     J gradients=J::array();J blockids=J::array();
     Eigen::Matrix<double,6,1> total=Eigen::Matrix<double,6,1>::Zero();double block_score=0;
     for(const auto& item:blocks) {
      auto block=matcher.evaluateResearchScore(item.second,q,false);total+=block.gradient;block_score+=block.score;
      gradients.push_back(matrix(block.gradient.transpose())[0]);blockids.push_back({std::get<0>(item.first),std::get<1>(item.first),std::get<2>(item.first),item.second.size()});
     }
     result["block_gradients"]=gradients;result["block_ids_and_counts"]=blockids;
     result["block_gradient_sum_error"]=(total-original.gradient).cwiseAbs().maxCoeff();
     result["block_score_sum_error"]=std::abs(block_score-original.score);
    }
   }
   result["wall_ms"]=std::chrono::duration<double,std::milli>(Clock::now()-begin).count();
   result["target_sha"]=target_identity;result["source_version"]=req.value("source_version",std::string("UNSPECIFIED"));
   std::cout<<result.dump()<<std::endl;
  }
 } catch(const std::exception& exc){std::cerr<<exc.what()<<std::endl;return 2;}
 return 0;
}
