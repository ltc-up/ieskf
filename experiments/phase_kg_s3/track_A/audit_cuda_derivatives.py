"""Host audit of production CUDA arithmetic, without redistributing its source.

Loads only the two pure arithmetic device functions into an isolated temporary
host shared object. No map, point cloud, GPU, or evaluation reference is used.
This is not a GPU equivalence or complete association-boundary derivative test.
"""
import argparse, ctypes, hashlib, json, pathlib, subprocess, tempfile
import numpy as np


def extract(text, name):
    start = text.index('__device__ void ' + name)
    brace = text.index('{', start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end].replace('__device__', '')


def run(source, corrected=False):
    raw = pathlib.Path(source).read_text()
    code = r'''
#include <cmath>
using std::isfinite;
struct float3 {float x,y,z;};
constexpr int kReductionValues=32;
struct DeviceTargetView {
 const float *mean_x,*mean_y,*mean_z,*icov_xx,*icov_xy,*icov_xz,*icov_yy,*icov_yz,*icov_zz;
};
int upperIndex(int row,int col) {if(row>col){int t=row;row=col;col=t;} return row*6-row*(row-1)/2+col-row;}
'''
    code += extract(raw, 'computeTransformAndDerivatives') + '\n'
    code += extract(raw, 'addVoxelContribution') + '\n'
    code += r'''
extern "C" void pair(const double* q, const double* x, const double* m,
                     double d1, double d2, double* out) {
 float3 p{(float)x[0],(float)x[1],(float)x[2]},y;
 float j[18],ph[15];computeTransformAndDerivatives(p,q,&y,j,ph);
 float a=(float)m[0],b=(float)m[1],c=(float)m[2],one=1,zero=0;
 DeviceTargetView t{&a,&b,&c,&one,&zero,&zero,&one,&zero,&one};
 for(int i=0;i<32;i++)out[i]=0;
 addVoxelContribution(t,0,y,j,ph,(float)d1,(float)d2,false,50,true,out);
}
'''
    if corrected:
        code=code.replace('computeTransformAndDerivatives(p,q,&y,j,ph);','computeTransformAndDerivatives(p,q,&y,j,ph,true);')
    with tempfile.TemporaryDirectory(prefix='kg_s3_math_') as temp:
        cpp=pathlib.Path(temp)/'audit.cpp';so=pathlib.Path(temp)/'audit.so'
        cpp.write_text(code)
        subprocess.run(['g++','-O2','-shared','-fPIC','-std=c++17',str(cpp),'-o',str(so)],check=True)
        lib=ctypes.CDLL(str(so));ptr=ctypes.POINTER(ctypes.c_double)
        lib.pair.argtypes=[ptr,ptr,ptr,ctypes.c_double,ctypes.c_double,ptr]
        # Nonzero roll/pitch/yaw and z keep this away from angle snaps. One
        # fixed Gaussian removes neighbor/q50 discontinuities entirely.
        q=np.array([.1,-.2,.3,.2,.3,-.4],dtype='d')
        point=np.array([.4,-.6,2.],dtype='d')
        mean=np.array([.2,-.1,.7],dtype='d')
        r=1.5;o=.55;c1=10*(1-o);c2=o/r**3;d3=-np.log(c2)
        d1=-np.log(c1+c2)-d3
        d2=-2*np.log((-np.log(c1*np.exp(-.5)+c2)-d3)/d1)
        def evaluate(z):
            out=np.zeros(32,dtype='d')
            lib.pair(z.ctypes.data_as(ptr),point.ctypes.data_as(ptr),mean.ctypes.data_as(ptr),d1,d2,out.ctypes.data_as(ptr))
            H=np.zeros((6,6));H[np.triu_indices(6)]=out[7:28];H=H+np.triu(H,1).T
            return out[0],out[1:7],H
        S,g,H=evaluate(q)
        rows=[]
        for h in [.01,.003,.001]:
            numerical_g=np.zeros(6);numerical_H=np.zeros((6,6))
            for k in range(6):
                dq=np.eye(6)[k]*h
                sp,gp,_=evaluate(q+dq);sm,gm,_=evaluate(q-dq)
                numerical_g[k]=(sp-sm)/(2*h)
                numerical_H[:,k]=(gp-gm)/(2*h)
            rows.append(dict(h=h,gradient_max_abs=float(abs(g-numerical_g).max()),hessian_max_abs=float(abs(H-numerical_H).max()),pitch_pitch_reported=float(H[4,4]),pitch_pitch_gradient_fd=float(numerical_H[4,4]),hessian_difference=(H-numerical_H).tolist()))
        grid=[]
        generator=np.random.default_rng(1234)
        # Same source/means and all 21 independent Hessian entries, several
        # pitches including signs and nonzero orientations. No zero-angle snap.
        for pitch in [-.6,-.2,.05,.3,.65]:
            for cloud_case in range(3):
                q[:]=[.1,-.2,.3,.13+.07*cloud_case,pitch,-.35+.09*cloud_case]
                point[:]=generator.uniform([-2,-2,-2],[2,2,2])
                mean[:]=generator.uniform([-.7,-.7,-.7],[.7,.7,.7])
                sv,gv,hv=evaluate(q);h=.003
                fd=np.zeros((6,6));fg=np.zeros(6)
                for j in range(6):
                    dq=np.eye(6)[j]*h
                    sp,gp,_=evaluate(q+dq);sm,gm,_=evaluate(q-dq)
                    fg[j]=(sp-sm)/(2*h);fd[:,j]=(gp-gm)/(2*h)
                grid.append(dict(pitch=pitch,cloud_case=cloud_case,score=sv,gradient=gv.tolist(),hessian=hv.tolist(),newton_direction=np.linalg.lstsq(hv,-gv,rcond=1e-10)[0].tolist(),gradient_max_abs=float(abs(gv-fg).max()),hessian_upper21_abs=abs(hv-fd)[np.triu_indices(6)].tolist(),hessian_max_abs=float(abs(hv-fd).max())))
        snapped=[]
        point[:]=[.4,-.6,2.];mean[:]=[.2,-.1,.7]
        for axis in [3,4,5]:
            q[:]=[.1,-.2,.3,.2,.3,-.4];q[axis]=0
            sv,gv,hv=evaluate(q);dq=np.eye(6)[axis]*2e-5
            sp,gp,_=evaluate(q+dq);sm,gm,_=evaluate(q-dq)
            snapped.append(dict(axis=int(axis),reported_gradient=float(gv[axis]),score_fd_gradient=float((sp-sm)/(4e-5)),h=2e-5,score_plus=float(sp),score_minus=float(sm)))
        result=dict(snapped_angle_cases=snapped,corrected_pitch=corrected,grid=grid,grid_max_H_error=max(z['hessian_max_abs'] for z in grid),source_sha256=hashlib.sha256(raw.encode()).hexdigest(),scope='exact extracted CUDA pair arithmetic compiled on host; fixed association',score=float(S),d1=float(d1),d2=float(d2),cases=rows,status='DERIVATIVE_INCONSISTENCY' if rows[-1]['hessian_max_abs']>.01 else 'SMOOTH_PASS_WITH_UNRESOLVED_SNAPPED_ANGLE_MISMATCH',production_source_modified=False)
        return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('cuda_source');p.add_argument('--out');p.add_argument('--corrected',action='store_true');a=p.parse_args()
    result=run(a.cuda_source,a.corrected);text=json.dumps(result,indent=2)
    if a.out:pathlib.Path(a.out).write_text(text+'\n')
    print(text)
