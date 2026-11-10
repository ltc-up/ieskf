// Independent research protocol adapter. Uses existing MIT small_gicp APIs.
// small_gicp Copyright (c) 2024 Kenji Koide; dependency is not vendored here.
#include <small_gicp/ann/kdtree_tbb.hpp>
#include <small_gicp/util/normal_estimation_tbb.hpp>
#include <small_gicp/registration/reduction_tbb.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <tbb/task_arena.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <chrono>
using Json=nlohmann::json;using Cloud=small_gicp::PointCloud;using Tree=small_gicp::KdTree<Cloud>;using Clock=std::chrono::steady_clock;
double elapsed(Clock::time_point t){return std::chrono::duration<double,std::milli>(Clock::now()-t).count();}
Json matrix(const Eigen::MatrixXd& m){Json a=Json::array();for(int i=0;i<m.rows();i++){Json b=Json::array();for(int j=0;j<m.cols();j++)b.push_back(m(i,j));a.push_back(b);}return a;}
std::shared_ptr<Cloud> load(const std::string& path){std::ifstream f(path,std::ios::binary);uint64_t n=0;f.read(reinterpret_cast<char*>(&n),8);if(n>20000000)throw std::runtime_error("POINT_LIMIT");std::vector<Eigen::Vector3f> p(n);for(auto& v:p)f.read(reinterpret_cast<char*>(v.data()),12);if(!f)throw std::runtime_error("INVALID_POINT_FILE");return std::make_shared<Cloud>(p);}
struct RoiRejector {double max_dist_sq=16;bool active=false;Eigen::Vector3d center=Eigen::Vector3d::Zero();double low=-1e9,high=1e9,radius=120;
 template<class A,class B> bool operator()(const A& a,const B&,const Eigen::Isometry3d&,size_t k,size_t,double d)const {Eigen::Vector3d p=small_gicp::traits::point(a,k).template head<3>();return d>max_dist_sq||(active&&((p-center).template head<2>().squaredNorm()>radius*radius||p.z()<low||p.z()>high));}
};
int main(){std::shared_ptr<Cloud> target;std::shared_ptr<Tree> tree;std::string identity;double target_ms=0; tbb::task_arena arena(4);std::string line;
 while(std::getline(std::cin,line)){try{Json q=Json::parse(line),out;arena.execute([&]{auto total=Clock::now();bool hit=target&&identity==q.at("target_sha").get<std::string>();if(!hit){auto t=Clock::now();target=load(q.at("target_path"));tree=std::make_shared<Tree>(target,small_gicp::KdTreeBuilderTBB());small_gicp::estimate_covariances_tbb(*target,*tree,20);target_ms=elapsed(t);identity=q.at("target_sha");}auto t=Clock::now();auto src=load(q.at("source_path"));out={{"case_id",q.at("case_id")},{"source_n",src->size()},{"target_cache_hit",hit},{"target_build_ms",hit?0:target_ms},{"backend","small_gicp"}};if(src->size()<20){out.update({{"converged",false},{"candidate_valid",false},{"exit","NO_CLOUD"}});return;}auto stree=std::make_shared<Tree>(src,small_gicp::KdTreeBuilderTBB());double st=elapsed(t);t=Clock::now();small_gicp::estimate_covariances_tbb(*src,*stree,20);double sc=elapsed(t);Eigen::Isometry3d seed=Eigen::Isometry3d::Identity();for(int i=0;i<4;i++)for(int j=0;j<4;j++)seed.matrix()(i,j)=q.at("seed_common")[i][j];
 small_gicp::Registration<small_gicp::GICPFactor,small_gicp::ParallelReductionTBB,small_gicp::NullFactor,RoiRejector> reg;reg.optimizer.max_iterations=50;reg.criteria.translation_eps=.001;reg.criteria.rotation_eps=.0017453292519943296;auto roi=q.at("roi");reg.rejector.active=roi[0];for(int i=0;i<3;i++)reg.rejector.center[i]=roi[i+2];reg.rejector.low=roi[9];reg.rejector.high=roi[10];reg.rejector.radius=roi[11];t=Clock::now();auto r=reg.align(*target,*src,*tree,seed);double align=elapsed(t);
 // Optimizer H may precede the final accepted step. Refresh actual final factors.
 t=Clock::now();std::vector<small_gicp::GICPFactor> factors(src->size());auto [H,g,cost]=reg.reduction.linearize(*target,*src,*tree,reg.rejector,r.T_target_source,factors);size_t pairs=std::count_if(factors.begin(),factors.end(),[](const auto&f){return f.inlier();});double info=elapsed(t);out.update({{"converged",r.converged},{"candidate_valid",r.T_target_source.matrix().allFinite()&&std::isfinite(cost)&&pairs>0},{"final",matrix(r.T_target_source.matrix())},{"iterations",r.iterations},{"exit",r.converged?"UPDATE_TOLERANCE":"NOT_CONVERGED"},{"native_cost",cost},{"pairs",pairs},{"H_right_rotation_translation",matrix(H)},{"gradient_right_rotation_translation",matrix(g)},{"optimizer_last_H",matrix(r.H)},{"fitness",nullptr},{"source_tree_ms",st},{"source_covariance_ms",sc},{"align_wall_ms",align},{"information_refresh_ms",info},{"service_wall_ms",elapsed(total)},{"statistical_covariance",false}});
 if(q.value("curvature_probe",false)){Json probes=Json::array();double e0=0;for(auto&f:factors)e0+=f.error(*target,*src,r.T_target_source);for(int j=0;j<6;j++){Eigen::Matrix<double,6,1>d=Eigen::Matrix<double,6,1>::Zero();d[j]=j<3?1e-5:1e-4;double ep=0,em=0;auto plus=r.T_target_source*small_gicp::se3_exp(d),minus=r.T_target_source*small_gicp::se3_exp(-d);for(auto&f:factors){ep+=f.error(*target,*src,plus);em+=f.error(*target,*src,minus);}probes.push_back({{"axis",j},{"step",d[j]},{"base",e0},{"plus",ep},{"minus",em},{"GN_diagonal",H(j,j)},{"gradient",g[j]}});}out["frozen_factor_curvature"]=probes;}
 });std::cout<<out.dump()<<std::endl;
 }catch(const std::exception&e){std::cout<<Json({{"candidate_valid",false},{"converged",false},{"exit",e.what()}}).dump()<<std::endl;}}
}
