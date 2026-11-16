"""Bounded original-frame CUDA diagnostic; no filter, GT seed or new cropping."""
import argparse,json,hashlib,struct,sys,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT/'experiments/phase_be_e0_e4'))
from runtime import Service,write_cloud
ORIGIN=None;LEGACY_ORIGIN=None

def read_pcd(path):
 with Path(path).open('rb') as f:
  h={}
  while True:
   line=f.readline().decode('ascii').strip()
   if not line:continue
   if line.startswith('#'):continue
   k,*v=line.split();h[k]=v
   if k=='DATA':break
  names=h['FIELDS'];sizes=list(map(int,h['SIZE']));types=h['TYPE'];counts=list(map(int,h.get('COUNT',['1']*len(names))));n=int(h['POINTS'][0]);fm={'F':'f','I':'i','U':'u'};dtype=np.dtype([(k,'<'+fm[t]+str(s),(c,)) if c!=1 else (k,'<'+fm[t]+str(s)) for k,s,t,c in zip(names,sizes,types,counts)])
  if h['DATA']==['binary']:
   b=f.read();assert len(b)==n*dtype.itemsize;a=np.frombuffer(b,dtype=dtype)
   xyz=np.column_stack([a[k] for k in ['x','y','z']]).astype('<f4')
  elif h['DATA']==['ascii']:
   a=np.loadtxt(f,ndmin=2);xyz=a[:,[names.index(k) for k in ['x','y','z']]].astype('<f4')
  else:raise ValueError('UNSUPPORTED_PCD_ENCODING')
 assert len(xyz)==n and np.isfinite(xyz).all();return xyz

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def prepare(root,archive):
 global ORIGIN,LEGACY_ORIGIN
 contract=json.load(open(Path(root)/'input_time_contract.json'));ORIGIN=int(contract['bag_origin_ns']);LEGACY_ORIGIN=int(contract['legacy_origin_ns'])
 root=Path(root)
 if (root/'frozen/requests.json').exists():raise FileExistsError('REUSE_EXISTING_IDENTITY_NO_OVERWRITE')
 archive=Path(archive);critical=archive/'fitness_semantics_20261003/runs/critical/gpu1013/captures';tent=archive/'tent_map_hypothesis';caps=sorted(p for p in critical.iterdir() if p.is_dir());ix=set(np.linspace(0,len(caps)-1,18,dtype=int).tolist());ix|={11,15,29};items=[]
 for i in sorted(ix):
  cap=caps[i];items.append(dict(group='713_original',header=int(cap.name),source=cap/'source_after_preprocess.pcd',target=cap/'active_target_raw.pcd',seed=cap/'seed_pose.txt',old_final=cap/'final_pose.txt',map_variant='1013_captured_raw'))
 old=sorted(p for p in (tent/'closed_loop_1013/captures').iterdir() if p.is_dir() and 395.5<=(int(p.name)-LEGACY_ORIGIN)/1e9<=397.5)
 old=[old[i] for i in sorted(set(np.linspace(0,len(old)-1,min(8,len(old)),dtype=int)))]
 for cap in old:items.append(dict(group='old_first_switch',header=int(cap.name),source=cap/'source_after_preprocess.pcd',target=cap/'active_target_raw.pcd',seed=cap/'seed_pose.txt',old_final=cap/'final_pose.txt',map_variant='1013_captured_raw'))
 controls=old[-3:]
 for cap in controls:
  dest=tent/'closed_loop_frozen/1013'/cap.name
  for v in ['1013','0105','1013_plus_tent']:
   f=dest/('target_'+v+'.pcd')
   if not f.exists():raise FileNotFoundError(f)
   items.append(dict(group='map_control',header=int(cap.name),source=cap/'source_after_preprocess.pcd',target=f,seed=cap/'seed_pose.txt',old_final=dest/('final_'+v+'.txt'),map_variant=v))
 assert len(items)<=40
 cfg=dict(resolution=1.5,step_size=.2,q=50,max_iterations=35,neighbor='DIRECT7',line_search='GLL3',transformation_epsilon=.01,relative_objective_epsilon=.001,fitness=True,source_resampled=False,extra_crop=False,history_parity='same original source/target/seed; old optimizer/q identity incomplete: current configuration separate, NOT historical numeric reproduction')
 (root/'frozen/selection_before_matching.json').write_text(json.dumps(dict(count=len(items),config=cfg,items=[{k:str(v) if isinstance(v,Path) else v for k,v in x.items()} for x in items]),indent=2))
 req=[]
 for i,it in enumerate(items):
  s=read_pcd(it['source']);target=read_pcd(it['target']);seed=np.loadtxt(it['seed']);assert seed.shape==(4,4) and np.allclose(seed[3],[0,0,0,1]);folder=root/'frozen'/f'{i:02d}';folder.mkdir();sp=folder/'source.bin';tp=folder/'target.bin';write_cloud(sp,s);write_cloud(tp,target)
  req.append(dict(case_id=f'C4_{i:02d}',header=it['header'],group=it['group'],scene='C4_frozen',source_path=str(sp),target_path=str(tp),target_sha=sha(tp),seed_common=seed.tolist(),roi=[False,1,0,0,0,0,0,0,0,-10000,10000,120,i+1],q=50.,max_iterations=35,bd_config=dict(transformation_epsilon=.01,relative_objective_epsilon=.001,line_search=True,mode='gll'),identity=dict(source_original_sha=sha(it['source']),target_original_sha=sha(it['target']),seed_original_sha=sha(it['seed']),source_n=len(s),target_n=len(target),map_variant=it['map_variant'],bag_relative=(it['header']-ORIGIN)*1e-9,old_relative=(it['header']-LEGACY_ORIGIN)*1e-9,old_final=np.loadtxt(it['old_final']).tolist() if it['old_final'].exists() else None)))
 (root/'frozen/requests.json').write_text(json.dumps(req,indent=2));return req

def run(root,binary):
 root=Path(root);req=json.load(open(root/'frozen/requests.json'));dest=root/'frozen/results.jsonl'
 if dest.exists():raise FileExistsError(dest)
 service=Service(binary,root/'logs/ndt_stderr.log');n=0
 try:
  with dest.open('x') as out:
   for r in req:
    begin=time.perf_counter_ns();v,wall=service.call(r);assert v['source_n']==r['identity']['source_n'];old=r['identity']['old_final'];diff=float(np.linalg.norm(np.array(v['final'])[:3,3]-np.array(old)[:3,3])) if old else None
    out.write(json.dumps(dict(request=r,result=v,service_wall_ms=wall,old_final_translation_difference=diff))+'\n');out.flush();n+=1;print(r['case_id'],r['group'],v['converged'],v['exit'],round(wall,3),flush=True)
 finally:service.close();(root/'frozen/run_count.json').write_text(json.dumps(dict(actual_registrations=n,max_authorized=40,no_filter_updates=True),indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('mode',choices=['prepare','run']);p.add_argument('root');p.add_argument('dependency');a=p.parse_args();prepare(a.root,a.dependency) if a.mode=='prepare' else run(a.root,a.dependency)
