#include "radius_kdtree.hpp"
#include <cassert>
#include <iostream>
#include <limits>
int main(){std::vector<float>x,y,z;std::vector<unsigned>count;
 for(int a=-3;a<=3;a++)for(int b=-3;b<=3;b++){x.push_back(a*.5f);y.push_back(b*.5f);z.push_back(0);count.push_back(6);}
 x.push_back(std::numeric_limits<float>::quiet_NaN());y.push_back(0);z.push_back(0);count.push_back(6);
 x.push_back(0);y.push_back(0);z.push_back(0);count.push_back(0);
 auto nodes=bd::build_radius_tree(x,y,z,count);int queries=0;
 for(float qx:{-.500001f,-.5f,-.499999f,0.f,.499999f,.5f,.500001f})for(float qy:{-.5f,0.f,.5f})for(float radius:{0.f,.5f,1.f,1.5f}){
  float p[3]={qx,qy,0};auto actual=bd::radius_search(nodes,x,y,z,p,radius);std::vector<int>expected;
  for(size_t i=0;i<x.size();i++){float dx=qx-x[i],dy=qy-y[i];if(count[i]&&std::isfinite(x[i])&&dx*dx+dy*dy<=radius*radius)expected.push_back(i);}
  assert(actual==expected);queries++;
 }
 std::cout<<"PASS "<<queries<<" radius queries: exact boundary/planar/negative-coordinate/invalid-voxel oracle\n";
}
