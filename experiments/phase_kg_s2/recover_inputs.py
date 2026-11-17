"""Bounded identity-checked reconstruction from original CDR and retained causal events.
No map matching or GNSS reader. Original artifacts remain read-only.
"""
import sys,json,pickle,hashlib,sqlite3,time,copy
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree
ROOT=Path(__file__).resolve().parents[2];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'experiments/phase_bg_far_lio/c5'),str(ROOT/'experiments/phase_bg_far_lio'),str(ROOT/'experiments/phase_be_e0_e4')]
import trace_backend
from input_adapter import decode_cloud,fuse_three
from runtime import Service,read_cloud,write_cloud,partition
from geometry_ieskf.external_pose import ExternalPose
from geometry_ieskf.common import C

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def base_pose(x,lever):
 T=np.eye(4);T[:3,:3]=x.R;T[:3,3]=x.p-x.R@lever;return T

def main(out,existing):
 from rclpy.serialization import deserialize_message
 from sensor_msgs.msg import PointCloud2
 out=Path(out);old=Path(existing);cfg=json.load(open(old/'phase_bg_far_lio/c5/config.json'));paths=json.load(open(old/'phase_bg_far_lio/c5/paths.json'));audit=json.load(open(cfg['audit']));Tbi=np.array(json.load(open(cfg['extrinsics']))['T_base_imu']);lever=Tbi[:3,3];B=cfg['start_ns']-130_000_000_000;dest=out/'inputs/fault';dest.mkdir(exist_ok=False);scratch=out/'runtime/deskew';scratch.mkdir();D=Service(paths['deskew'],out/'logs/deskew.log',(scratch/'meta.jsonl',scratch));assert D.proc.stdout.readline().strip()=='READY';metadata=(scratch/'meta.jsonl').open();db=sqlite3.connect('file:'+cfg['bag']+'?mode=ro&immutable=1',uri=True);edges={e['child']:e for e in audit['tf_static']}
 def tf(child):
  if child=='base_link':return np.eye(4)
  e=edges[child];z=np.eye(4);z[:3,:3]=Rotation.from_quat(e['quaternion']).as_matrix();z[:3,3]=e['translation'];return tf(e['parent'])@z
 transforms=[tf(s)for s in ['lidar_front','lidar_left','lidar_right']];c5=old/'phase_bg_far_lio/c5/eskf15_1013';M={r['stamp_ns']:r for r in map(json.loads,open(c5/'matches.jsonl')) if 'ndt'in r and 515<=(r['stamp_ns']-B)/1e9<=520};replay=pickle.load(open(c5/'deterministic_checkpoint.pkl','rb'));records=[];reason='UNKNOWN';verified=0
 try:
  for rec in map(json.loads,open(c5/'kernel_events.jsonl')):
   e=rec['event']
   if e['kind']=='ndt':e['pose']=np.array(e['pose']);e['R']=np.array(e['R']);e['observer']=ExternalPose(lever,('x','y','z','rx','ry','rz'))
   replay.ingest(e);x,Q,_=replay.current();h=hashlib.sha256(np.r_[x.p,x.v,x.R.ravel(),x.bg,x.ba,Q.ravel()].astype('<f8').tobytes()).hexdigest();assert h==rec['state_hash'],'REPLAY_STATE_IDENTITY'
   if e['kind']!='clock' or e['header_ns']not in M:continue
   r=M[e['header_ns']];ref=r['stamp_ns'];cl=[];raw_ids=r['input_meta']['raw_identities']
   for sid,ident in enumerate(raw_ids):
    row=db.execute('select timestamp,data from messages where id=?',(ident['id'],)).fetchone();assert row and hashlib.sha256(row[1]).hexdigest()==ident['cdr_sha256'];m=deserialize_message(row[1],PointCloud2);head=m.header.stamp.sec*10**9+m.header.stamp.nanosec;fields={f.name:dict(offset=f.offset,datatype=f.datatype,count=f.count)for f in m.fields};cl.append(decode_cloud(m.data,fields,m.width*m.height,m.point_step,head,row[0],sid,ident['id'],transforms[sid],m.is_bigendian))
   f,meta=fuse_three(cl);assert meta['header_ns']==ref and meta['available_ns']==r['available_ns'];span=r['input_meta']['point_start_ns'];hist=sorted({s.t:s for s,_,_ in [replay.base]+replay.snapshots if span-20_000_000<=s.t<=ref}.values(),key=lambda s:s.t);prior,Q,im=next(z for z in reversed(replay.snapshots)if z[0].t==ref);seed=base_pose(prior,lever);assert np.max(abs(seed-np.array(r['seed'])))<1e-12
   clouds=[]
   for sid in range(3):
    z=f[f['sensor_id']==sid];p=scratch/f'sensor{sid}.raw';p.write_bytes(z.tobytes());fields={k:dict(offset=z.dtype.fields[k][1],datatype=7,count=1)for k in ['x','y','z','timestamp','intensity']};clouds.append(dict(raw_path=str(p),fields=fields,width=len(z),height=1,row_step=len(z)*z.dtype.itemsize,point_step=z.dtype.itemsize,bigendian=False,header_ns=ref,T_body_lidar=np.eye(4).tolist()))
   rows=[]
   for a in hist:
    T=base_pose(a,lever);rows.append(np.r_[(a.t-ref)*1e-9,Rotation.from_matrix(T[:3,:3]).as_quat(),T[:3,3]])
   poses=scratch/'poses.bin';np.asarray(rows,'<f8').tofile(poses);req=dict(header=0,native_reference_ns=ref,scene='KG_S2_identity',clouds=clouds,pose_path=str(poses),T_odom_native=seed.tolist(),T_common_from_native=np.eye(4).tolist(),gates_pass=True);tick=time.perf_counter_ns();D.proc.stdin.write(json.dumps(req)+'\n');D.proc.stdin.flush();assert D.proc.stdout.readline().strip()=='DONE';ds=json.loads(metadata.readline());p=read_cloud(ds['source_path']);source_sha=sha(ds['source_path']);assert source_sha==r['source_sha'],'SOURCE_BYTES_DIFFER_FROM_C5';sp=dest/f'{ref}.bin';write_cloud(sp,p)
   # Proxy ownership only: voxel centroids may mix sensors. Never claim exact independent ray counts.
   raw=np.fromfile(ds['common_xyzi'],dtype='<f4').reshape(-1,4);assert len(raw)==len(f),'RAW_ORDER_COUNT';owners=np.repeat(np.arange(3),[int((f['sensor_id']==sid).sum())for sid in range(3)]);dd,nn=cKDTree(raw[:,:3]).query(p,workers=1);np.save(dest/f'{ref}_owner_proxy.npy',owners[nn]);verified+=1;records.append(dict(t=ref,available=r['available_ns'],source=str(sp),source_sha=source_sha,source_n=len(p),raw_identities=raw_ids,point_start_ns=span,source_matches_C5=True,prior=dict(p=prior.p.tolist(),v=prior.v.tolist(),R=prior.R.tolist(),bg=prior.bg.tolist(),ba=prior.ba.tolist(),P=Q.tolist()),seed=seed.tolist(),owner_proxy=str(dest/f'{ref}_owner_proxy.npy'),owner_proxy_distance_max=float(dd.max()),deskew_ms=(time.perf_counter_ns()-tick)/1e6,legacy=r['ndt'],legacy_update=r['update']))
   print('verified',verified,round((ref-B)/1e9,6),len(p),flush=True)
   if len(records)==len(M):break
  assert len(records)==len(M);reason='COMPLETE_BYTE_IDENTICAL_TO_C5'
 finally:
  D.close();db.close();metadata.close();(out/'inputs/fault_status.json').write_text(json.dumps(dict(reason=reason,verified=verified,expected=len(M)),indent=2));(out/'inputs/fault_records.json').write_text(json.dumps(records,indent=2))
 if reason!='COMPLETE_BYTE_IDENTICAL_TO_C5':raise RuntimeError(reason)
 # Existing normal/control source histories, same deterministic training split as original A.
 frames=list(map(json.loads,open(old/'phase_be_e0_e4/results/controlled2_matched_lap/frames.jsonl')));folder=old/'phase_be_e0_e4/results/controlled2_matched_lap/deskew_ndt';series=[dict(id='fault',records=records)]
 for label,lo,hi in [('normal8',700,720),('right3',1340,1360),('ordinary',200,220),('large_proxy',120,140)]:
  seq=[];sd=out/'inputs'/label;sd.mkdir()
  for row in frames[lo:hi+1]:
   t=row['ref'];src=folder/f'{t}_A1.bin';a=read_cloud(src);a=partition(a)[0];b=row['branches'][0];assert len(a)==b['ndt']['source_n'];sp=sd/f'{t}.bin';write_cloud(sp,a);pr=b['prior'];R=np.array(pr['R']);T=np.eye(4);T[:3,:3]=R;T[:3,3]=np.array(pr['p'])-R@lever;seq.append(dict(t=t,available=row['available'],source=str(sp),source_sha=sha(sp),source_n=len(a),original_prepartition_sha=sha(src),prior=pr,seed=T.tolist(),legacy=b['ndt'],owner_proxy=None,source_contract='original BE causal own15D Deskew plus original deterministic holdout, not current graph feedback'))
  series.append(dict(id=label,records=seq))
 # Another pass retains original captures; no claim of a recovered15D checkpoint.
 c4=old/'phase_bg_far_lio/c4';sel=json.load(open(c4/'frozen/selection_before_matching.json'))['items'];caps=Path(sel[0]['source']).parent.parent;sys.path.insert(0,str(ROOT/'experiments/phase_bg_far_lio/c4'));from frozen_diagnostic import read_pcd
 motion=json.load(open(paths['motion']));events=motion['events'];seq=[];sd=out/'inputs/pass713';sd.mkdir()
 for cap in sorted(caps.iterdir()):
  if not cap.is_dir()or not cap.name.isdigit():continue
  t=int(cap.name);a=read_pcd(cap/'source_after_preprocess.pcd');sp=sd/f'{t}.bin';write_cloud(sp,a);seq.append(dict(t=t,available=None,source=str(sp),source_sha=sha(sp),source_n=len(a),seed=np.loadtxt(cap/'seed_pose.txt').tolist(),owner_proxy=None,source_contract='original captured historical production Deskew; exact original recorder availability unavailable'))
 series.append(dict(id='pass713',records=seq,qualification='offline_only_original_availability_UNKNOWN; no causal shadow timing claim'))
 document=dict(baseline='d418fa2c5c996ec859e310eb2321d8abfc47fe6c',origin=B,T_base_imu=Tbi.tolist(),noise=C,series=series,global_maps=dict(map1013=paths['target'],map0105=str(old/'phase_bd_ndt_health_bridge/inputs/research_data/phase_ar_loop44_map0105/map0105_target.bin')),motion=paths['motion'],config=cfg,lag_s=.5,vgicp_resolution=1.,cov_neighbors=20,geom_cov_eigen=[.001,1,1],LM_max_iterations=8,Huber_white_norm_delta=2.795,window_kind='causal_order_backend_shadow_and_retrospective_smoothing; frozen original deskew',no_current_graph_deskew_feedback=True)
 (out/'inputs/series.json').write_text(json.dumps(document,indent=2));print('all series prepared')
if __name__=='__main__':main(*sys.argv[1:])
