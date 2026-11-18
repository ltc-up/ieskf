"""Apply the narrowly approved research-only Score API to an ignored copy.

The input must be a copy, never the production checkout. No restricted source
is included here. Existing align() retains default false correction exactly.
"""
from pathlib import Path
import argparse

def patch(root):
 root=Path(root).resolve()
 if 'data/private' not in str(root):raise ValueError('PRIVATE_COPY_REQUIRED')
 cu=root/'cuda/ndt_linearize.cu';h=root/'cuda/ndt_linearize.cuh'
 s=cu.read_text()
 s=s.replace('float3* transformed, float j[18], float ph[15]) {','float3* transformed, float j[18], float ph[15], bool corrected_pitch = false) {')
 old='ph[6] = static_cast<float>(-cy * cz * x + cy * sz * y + sy * z);'
 assert s.count(old)==1
 s=s.replace(old,'ph[6] = static_cast<float>(-cy * cz * x + cy * sz * y + (corrected_pitch ? -sy : sy) * z);')
 s=s.replace('double* block_sums) {','double* block_sums, bool corrected_pitch) {',1)
 s=s.replace('computeTransformAndDerivatives(source[i], pose, &transformed, j, ph);','computeTransformAndDerivatives(source[i], pose, &transformed, j, ph, corrected_pitch);',1)
 s=s.replace('LinearizationHost* host_result) {','LinearizationHost* host_result, bool corrected_pitch) {',1)
 s=s.replace('buffers.block_sums.data());\n  NDT_CUDA_CHECK(cudaGetLastError());','buffers.block_sums.data(), corrected_pitch);\n  NDT_CUDA_CHECK(cudaGetLastError());',1)
 cu.write_text(s)
 s=h.read_text().replace('LinearizationHost* host_result);','LinearizationHost* host_result, bool corrected_pitch = false);');h.write_text(s)
 h=root/'include/ndt_gpu_matcher/ndt_gpu_matcher.hpp';s=h.read_text()
 pos=s.index('class NdtGpuMatcher')
 s=s[:pos]+'''struct ResearchScore {
 double score = 0;
 Eigen::Matrix<double,6,1> gradient = Eigen::Matrix<double,6,1>::Zero();
 Eigen::Matrix<double,6,6> hessian = Eigen::Matrix<double,6,6>::Zero();
 unsigned long long valid_pairs = 0;
};
\n'''+s[pos:]
 s=s.replace('  /**\n   * @brief 更新收敛阈值', '''  // Synchronous, serialized research API. Explicit RxRyRz coordinates;
  // corrected_pitch applies only here, never to original align().
  ResearchScore evaluateResearchScore(const PointCloud & source,
    const Eigen::Matrix<double,6,1> & pose, bool corrected_pitch = true);

  /**
   * @brief 更新收敛阈值''')
 h.write_text(s)
 cpp=root/'src/ndt_gpu_matcher.cpp';s=cpp.read_text();pos=s.index('void NdtGpuMatcher::updateConvergenceCriteria')
 s=s[:pos]+'''ResearchScore NdtGpuMatcher::evaluateResearchScore(const PointCloud & source,
  const Eigen::Matrix<double,6,1> & pose, bool corrected_pitch)
{
 if (!impl_->target_ready || source.empty() || !pose.allFinite())
   throw std::invalid_argument("INVALID_RESEARCH_SCORE_INPUT");
 cuda_detail::uploadSource(impl_->linearization, source, impl_->stream_match);
 cuda_detail::LinearizationHost raw;
 cuda_detail::linearize(impl_->linearization, source.size(), impl_->activeTargetView(),
   pose.data(), impl_->config, impl_->stream_match, &raw, corrected_pitch);
 ResearchScore out; out.score=raw.score; out.valid_pairs=raw.valid_pairs;
 for(int i=0;i<6;++i) out.gradient[i]=raw.gradient[i];
 int k=0;for(int i=0;i<6;++i)for(int j=i;j<6;++j)
   out.hessian(i,j)=out.hessian(j,i)=raw.hessian[k++];
 return out;
}
\n'''+s[pos:];cpp.write_text(s)

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('root');patch(p.parse_args().root)
