"""Synthetic-only GPU interface, directional derivative and isolation audit."""
import argparse,json,struct,subprocess,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

def main(binary,out,extent=2):
 out=Path(out);out.mkdir(parents=True,exist_ok=False)
 rng=np.random.default_rng(9301);target=[]
 for i in range(-extent,extent+1):
  for j in range(-extent,extent+1):
   for k in range(2):
    center=np.array([i*3+.75,j*3+.75,k*3+.75])
    target.append(center+rng.normal(size=(32,3))*[.12,.08,.1])
 target=np.vstack(target);source=target[::5]
 for name,cloud in [('target',target),('source',source)]:
  (out/f'{name}.bin').write_bytes(struct.pack('<Q',len(cloud))+np.asarray(cloud,'<f4').tobytes())
 proc=subprocess.Popen([binary],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=(out/'service.stderr').open('w'),text=True)
 base=dict(source_path=str(out/'source.bin'),target_path=str(out/'target.bin'),target_sha='synthetic-fixed-grid',source_version='synthetic-source-1',roi=[False,1,0,0,0,0,0,0,0,-100,100,120,1])
 def call(**kwargs):
  proc.stdin.write(json.dumps(base|kwargs)+'\n');proc.stdin.flush();line=proc.stdout.readline()
  if not line:raise RuntimeError('SERVICE_EOF')
  return json.loads(line)
 allresults=[]
 for pitch in [-.018,-.006,.011]:
  q=np.array([.015,-.02,.01,.013,pitch,-.009]);v=call(op='score',pose_q=q.tolist())
  for h in [.0003,.0001]:
   fdg=np.zeros(6);fdH=np.zeros((6,6));paircounts=[]
   for k in range(6):
    d=np.eye(6)[k]*h;p=call(op='score',pose_q=(q+d).tolist());m=call(op='score',pose_q=(q-d).tolist())
    fdg[k]=(p['score']-m['score'])/(2*h)
    fdH[:,k]=(np.asarray(p['gradient'])-np.asarray(m['gradient']))/(2*h)
    paircounts.extend([p['valid_pairs'],m['valid_pairs']])
   original=np.asarray(v['original']['hessian']);corrected=np.asarray(v['hessian']);delta=abs(corrected-fdH)
   allresults.append(dict(pitch=pitch,h=h,score=v['score'],gradient=v['gradient'],original_hessian=original.tolist(),corrected_hessian=corrected.tolist(),numerical_hessian=fdH.tolist(),upper21_abs_error=delta[np.triu_indices(6)].tolist(),H_scaled_max_error=float((delta/(1+abs(corrected))).max()),H_relative_fro_error=float(np.linalg.norm(delta)/np.linalg.norm(corrected)),gradient_relative_error=float(np.linalg.norm(np.asarray(v['gradient'])-fdg)/(1+np.linalg.norm(v['gradient']))),S_g_exact_original_corrected=v['original']['score']==v['corrected']['score'] and v['original']['gradient']==v['corrected']['gradient'],valid_pair_counts=paircounts))
 q=np.array([.015,-.02,.01,.013,.011,-.009]);T=np.eye(4);T[:3,:3]=Rotation.from_euler('XYZ',q[3:]).as_matrix();T[:3,3]=q[:3]
 a=call(op='align',seed_common=T.tolist());call(op='score',pose_q=q.tolist(),corrected_pitch=True);b=call(op='align',seed_common=T.tolist())
 block=call(op='score',pose_q=q.tolist(),spatial_blocks=True,corrected_pitch=False)
 a1_update=None
 if len(block['block_gradients'])>=7:
  import sys
  sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'src'))
  from geometry_ieskf.state import State
  from score_map import DirectionalObserver
  prior=State(q[:3].copy(),np.zeros(3),T[:3,:3].copy(),np.zeros(3),np.zeros(3),0)
  def evaluate(v):return call(op='score',pose_q=np.asarray(v).tolist(),spatial_blocks=True,corrected_pitch=False)
  obs=DirectionalObserver(np.zeros(3),evaluate,'synthetic-source','synthetic-target')
  y,Q,info=obs.correct(prior,np.eye(15)*.04,np.asarray(a['final']),None)
  a1_update=info|dict(fullP_eigen_min=float(np.linalg.eigvalsh(Q).min()),fullP_shape=list(Q.shape))
 proc.stdin.close();proc.wait(timeout=20)
 result=dict(a1_update=a1_update,extent=extent,scope='synthetic GPU only, no accuracy or realtime claim',cases=allresults,align_result_exact_before_after_corrected_API=a['final']==b['final'] and a['score_hessian']==b['score_hessian'] and a['optimizer_score']==b['optimizer_score'],align_original=a,align_after=b,block_count=len(block['block_gradients']),block_gradient_sum_error=block['block_gradient_sum_error'],block_score_sum_error=block['block_score_sum_error'],passed=all(c['S_g_exact_original_corrected'] and c['H_relative_fro_error']<.003 for c in allresults) and a['final']==b['final'])
 (out/'results.json').write_text(json.dumps(result,indent=2)+'\n')
 print(json.dumps({k:v for k,v in result.items() if k not in ('cases','align_original','align_after')},indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('binary');p.add_argument('out');p.add_argument('--extent',type=int,default=2);a=p.parse_args();main(a.binary,a.out,a.extent)
