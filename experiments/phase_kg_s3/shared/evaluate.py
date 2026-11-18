"""Offline GNSS/INS engineering evaluation. NEVER imported by replay/optimizer.

Preserves KG-S2 reference reconstruction and 0/75/150 ms sensitivity; no fitted
alignment or best-offset selection. Longitudinal/lateral axes are reference-body
axes (not claimed identical to historical track-tangent scoring).
"""
import argparse,collections,hashlib,json,subprocess,sys
from pathlib import Path
import numpy as np,yaml
from scipy.spatial.transform import Rotation,Slerp
REPO=Path(__file__).resolve().parents[3];sys.path.insert(0,str(REPO/'src'))
from ndt_global.validation import static_transform
from ndt_global.track_evaluation import interpolate_reference
from ndt_global.footprint_audit import antenna_to_base
from provenance import ROOT,ARCHIVE,digest,require_run_output
from log_io import open_text


def stats(values):
 a=np.asarray(values,float)
 return dict(n=len(a),rmse=float(np.sqrt(np.mean(a*a))),p50_abs=float(np.median(abs(a))),p95_abs=float(np.percentile(abs(a),95)),p99_abs=float(np.percentile(abs(a),99)),max_abs=float(abs(a).max()),mean_signed=float(a.mean())) if len(a) else dict(n=0)

class EngineeringReference:
 def __init__(self,origin):
  self.origin=origin
  rawfile=ARCHIVE/'phase_bd_ndt_health_bridge/inputs/research_data/phase_at_gnss_footprint_corners/reference_bound.json'
  cfgfile=ARCHIVE/'phase_be_e0_e4/runtime/offline_reference_config.yaml'
  raw=json.loads(rawfile.read_text());cfg=yaml.safe_load(cfgfile.read_text());cfg=cfg.get('/**',{}).get('ros__parameters',cfg)
  ar=raw['/vectornav/raw/attitude'];at=np.array([r['header']-origin*1e-9 for r in ar]);q=np.array([r['q'] for r in ar]);keep=np.r_[True,np.diff(at)>0];self.at=at[keep];q=q[keep]
  self.T=static_transform(raw['/tf_static'],'base_link','vectornav_ant1')
  rot=Rotation.from_euler('xyz',cfg['pose_orientation_bias'])*Rotation.from_quat([2**-.5,2**-.5,0,0])*Rotation.from_quat(q)*Rotation.from_quat([1,0,0,0])*Rotation.from_matrix(self.T[:3,:3]).inv()
  self.slerp=Slerp(self.at,rot)
  gps=[r for r in raw['/vectornav/raw/gps'] if r['fix']==8];self.gt=np.array([r['header']-origin*1e-9 for r in gps])
  payload=''.join(' '.join(map(str,r['lla']))+'\n' for r in gps)
  z=subprocess.run(['CartConvert','-l',*map(str,cfg['reference_point']),'-p','10'],input=payload,text=True,capture_output=True,check=True)
  self.enu=np.loadtxt(z.stdout.splitlines());self.provenance=dict(raw_reference_sha256=digest(rawfile),offline_config_sha256=digest(cfgfile),independent_truth=False,alignment_fit=False,axes='GNSS_INS_reference_body',yaw_shifted_with_GNSS=False)
 def pose(self,t_ns,lag):
  qt=np.array([(int(t_ns)-self.origin)*1e-9])
  if not self.at[0]<=qt[0]<=self.at[-1]:return None
  k=np.searchsorted(self.at,qt[0])
  if 0<k<len(self.at) and self.at[k]-self.at[k-1]>.1:return None
  R=self.slerp(qt).as_matrix();ant,valid=interpolate_reference(self.gt-lag,self.enu,np.ones(len(self.gt),bool),qt,.600001)
  if not valid[0]:return None
  T=np.eye(4);T[:3,:3]=R[0];T[:3,3]=antenna_to_base(ant,R,self.T[:3,3])[0];return T

def errors(A,B):
 d=np.linalg.inv(B)@A
 return dict(xy=float(np.linalg.norm(d[:2,3])),longitudinal=float(d[0,3]),lateral=float(d[1,3]),yaw_deg=float(np.rad2deg(np.arctan2(d[1,0],d[0,0]))),z=float(d[2,3]))

def evaluate(run):
 run=Path(run);cfg=json.loads((run/'config.frozen.json').read_text());out=run/'evaluation';require_run_output(out);out.mkdir(exist_ok=False)
 origin=cfg['start_ns']-130*10**9;ref=EngineeringReference(origin)
 oldref=np.load(ARCHIVE/'phase_be_e0_e4/runtime/reference_lap.npz');grid=oldref['header']
 windows={'all':(135.,(cfg['end_ns']-origin)*1e-9),'fault':(515.,525.),'pass713':(705.,725.),'normal8':((int(grid[675])-origin)*1e-9,(int(grid[803])-origin)*1e-9),'right3':((int(grid[1317])-origin)*1e-9,(int(grid[1405])-origin)*1e-9)}
 values=collections.defaultdict(lambda:collections.defaultdict(list));reasons=collections.Counter();Bstats=collections.defaultdict(collections.Counter);timing=collections.defaultdict(list);pmin=[];asym=[];identity_bad=[];framecount=0;anchor_UO={};lastB={};badsegments=collections.defaultdict(list);active_bad={}
 details=(out/'errors.jsonl').open('x')
 for line in open_text(run/'frames.jsonl'):
  row=json.loads(line)
  if 'ndt' not in row:continue
  framecount+=1;t=row['ref_ns'];bag_s=(t-origin)*1e-9;reasons[row['update']['reason']]+=1
  for key in ['full_cycle_ms','deskew_ms','align_ms','delayed_update_ms']:timing[key].append(row[key])
  P=np.asarray(row['current']['P']);pmin.append(float(np.linalg.eigvalsh(P).min()));asym.append(float(abs(P-P.T).max()))
  poses={'seed':(t,np.asarray(row['seed'])),'ndt':(t,np.asarray(row['ndt']['final'])),'state':(row['current']['t'],np.asarray(row['current']['T_map_base']))}
  b=row.get('B')
  if b:
   if b['source_identity']!=row['source_payload_sha256']:identity_bad.append(t)
   for name,x in b['variants'].items():
    Bstats[name][x['status']]+=1;timing[name+'_callback_ms'].append(x.get('full_callback_variant_ms',x['full_step_ms']))
    if x['status']=='INITIALIZED':anchor_UO[name]=np.linalg.inv(np.asarray(x['O_T_U']))
    if name not in anchor_UO:raise ValueError('MISSING_LOCAL_INITIAL_ALIGNMENT')
    poses[name]=(t,anchor_UO[name]@np.asarray(x['O_T_base']))
    if name=='B0' and name in lastB and x['targets'] and x['targets'][0]['frame_id']!=lastB[name]:raise ValueError('B0_NOT_STRICT_PREVIOUS')
    lastB[name]=x['frame_id']
  record=dict(t=t,bag_s=bag_s,by_lag={})
  for lag in [0.,.075,.15]:
   comparisons={}
   for name,(stamp,A) in poses.items():
    B=ref.pose(stamp,lag)
    if B is None:continue
    e=errors(A,B);comparisons[name]=e
    for w,(lo,hi) in windows.items():
     if lo<=bag_s<=hi:
      for axis,v in e.items():values[(w,str(lag),name)][axis].append(v)
    # Diagnostic duration is explicitly an engineering discrepancy criterion,
    # not a true wrong-basin classifier. No value enters online selection.
    if lag==.075:
     bad=e['xy']>3.
     if bad and name not in active_bad:active_bad[name]=t
     elif not bad and name in active_bad:badsegments[name].append([active_bad.pop(name),t])
   record['by_lag'][str(lag)]=comparisons
  details.write(json.dumps(record)+'\n')
 details.close()
 for name,start in active_bad.items():badsegments[name].append([start,t])
 report=dict(processed=framecount,reference=ref.provenance,windows=windows,axes='reference_body',summary=[dict(window=w,lag=l,output=name,metrics={k:stats(v) for k,v in e.items()}) for (w,l,name),e in values.items()],update_reasons=dict(reasons),B_status={k:dict(v) for k,v in Bstats.items()},timing={k:stats(v) for k,v in timing.items()},P=dict(min_eigenvalue=min(pmin,default=None),max_asymmetry=max(asym,default=None),calibrated=False),source_identity_mismatches=identity_bad,engineering_discrepancy_over3m_segments={k:dict(n=len(v),maximum_duration_s=max(((b-a)*1e-9 for a,b in v),default=0),intervals=v) for k,v in badsegments.items()},performance_scope='serial full callback measurements; no live100Hz scheduling certification; grouped B callback cost is not separate process full-cycle qualification')
 (out/'summary.json').write_text(json.dumps(report,indent=2));print(json.dumps(dict(processed=framecount,update_reasons=dict(reasons),B_status=report['B_status'],P=report['P'],source_identity_mismatches=len(identity_bad))))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('run');evaluate(p.parse_args().run)
