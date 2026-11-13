#pragma once
// Independent research implementation. No production matcher source is vendored.
#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <vector>
namespace bd {
struct RadiusNode { int voxel,left,right,axis; float split; };
inline std::vector<RadiusNode> build_radius_tree(const std::vector<float>&x,
 const std::vector<float>&y,const std::vector<float>&z,const std::vector<unsigned>&count) {
 const auto n=x.size();
 if(y.size()!=n||z.size()!=n||count.size()!=n||n>=(size_t(1)<<30))throw std::invalid_argument("target shape/size");
 std::vector<int> ids;ids.reserve(n);
 for(size_t i=0;i<n;++i)if(count[i]&&std::isfinite(x[i])&&std::isfinite(y[i])&&std::isfinite(z[i]))ids.push_back(i);
 std::vector<RadiusNode> nodes;nodes.reserve(ids.size());
 auto coordinate=[&](int i,int a){return a==0?x[i]:a==1?y[i]:z[i];};
 std::function<int(int,int,int)> build=[&](int lo,int hi,int depth)->int {
  if(lo>=hi)return -1;const int axis=depth%3,mid=(lo+hi)/2;
  std::nth_element(ids.begin()+lo,ids.begin()+mid,ids.begin()+hi,[&](int a,int b){float u=coordinate(a,axis),v=coordinate(b,axis);return u==v?a<b:u<v;});
  int id=nodes.size(),voxel=ids[mid];nodes.push_back({voxel,-1,-1,axis,coordinate(voxel,axis)});
  int left=build(lo,mid,depth+1),right=build(mid+1,hi,depth+1);nodes[id].left=left;nodes[id].right=right;return id;
 };
 build(0,ids.size(),0);return nodes;
}
inline std::vector<int> radius_search(const std::vector<RadiusNode>&nodes,
 const std::vector<float>&x,const std::vector<float>&y,const std::vector<float>&z,
 const float point[3],float radius) {
 if(!std::isfinite(radius)||radius<0)throw std::invalid_argument("radius");
 std::vector<int> stack,result;if(!nodes.empty())stack.push_back(0);float r2=radius*radius;
 while(!stack.empty()) {
  auto node=nodes[stack.back()];stack.pop_back();float diff=point[node.axis]-node.split;
  int near=diff<0?node.left:node.right,far=diff<0?node.right:node.left;
  if(far>=0&&diff*diff<=r2)stack.push_back(far);if(near>=0)stack.push_back(near);
  int i=node.voxel;float dx=point[0]-x[i],dy=point[1]-y[i],dz=point[2]-z[i];
  if(dx*dx+dy*dy+dz*dz<=r2)result.push_back(i);
 }
 std::sort(result.begin(),result.end());return result;
}
} // namespace bd
