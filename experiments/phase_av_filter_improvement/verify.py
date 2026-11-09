"""Audit immutable predecessors and actual posterior arithmetic; export aggregates only."""
import argparse,hashlib,json,subprocess
from pathlib import Path
import numpy as np
from geometry_ieskf.state import State,chart_map
from geometry_ieskf.external_pose import ExternalPose
from geometry_ieskf.robust_packet_pose import scale_mixture_moments
from ndt_global.validation import static_transform
BASE='c647ac3224adee775d30dc6f9553c9b83612c060'
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(8*1024*1024),b''):h.update(b)
 return h.hexdigest()
def main(a):
 au=json.loads(Path('experiments/phase_au_prediction_feedback/manifest.json').read_text());at=json.loads(Path('experiments/phase_at_gnss_footprint_corners/manifest.json').read_text());binding=json.loads(Path('data/phase_at_gnss_footprint_corners/bindings.json').read_text());checks={}
 def check(name,p,expected):
  actual=sha(p);assert actual==expected,name;checks[name]=actual
 for k,v in at['input_sha256'].items():check('AT_'+k,binding[k],v)
 for name,p in [('AR_transactions','data/phase_ar_loop44_map0105/fullpose100_lap/transactions.json'),('AS_F_transactions','data/phase_as_accuracy_quality/F_multilap/transactions.json'),('AS_N_transactions','data/phase_as_accuracy_quality/N_lap/transactions.json'),('AT_causal',str(Path(binding['private'])/'causal.npz')),('frozen_config','data/history/phase_v/config/frozen.json')]:check(name,p,au['verified_input_sha256'][name])
 for name,v in au['actual_private_sha256'].items():check('AU_'+name,Path('data/phase_au_prediction_feedback')/name,v)
 files=subprocess.check_output(['git','ls-tree','-r','--name-only',BASE,'--','src','experiments/phase_ar_loop44_map0105','experiments/phase_as_accuracy_quality','experiments/phase_at_gnss_footprint_corners','experiments/phase_au_prediction_feedback'],text=True).splitlines()
 for p in files:assert Path(p).read_bytes()==subprocess.check_output(['git','show',BASE+':'+p]),p
 ar=json.loads(Path('data/phase_ar_loop44_map0105/fullpose100_lap/transactions.json').read_text())[:40];reproduction={}
 for run in ['baseline40','legacy_switch40']:
  tx=json.loads(Path('data/phase_av_filter_improvement',run,'transactions.json').read_text());diff={}
  for k in ['seed','prior.P','ndt.final','update.delta','update.K']:
   def val(r):
    for v in k.split('.'):r=r[v]
    return np.asarray(r)
   diff[k]=max(float(np.max(abs(val(x)-val(y)))) for x,y in zip(ar,tx))
  assert len(tx)==40 and not any(diff.values());reproduction[run]=diff
 tx=json.loads((a.run/'transactions.json').read_text());lever=static_transform(json.loads(Path('data/phase_ao_ap_validation/loop44.json').read_text())['transforms'],'base_link','imu_bosch')[:3,3];delta_error=max(float(np.max(abs(np.asarray(r['update']['K'])@r['update']['innovation']-r['update']['delta']))) for r in tx if r['update']['accepted']);Qerr=meanerr=0.;selected=0
 for r in tx:
  if not r['update']['accepted'] or (r['index']%19 and not r['ndt'].get('av_projection')):continue
  u=r['update'];x=State(**{k:np.asarray(v) for k,v in u['native_state_before'].items()});res,H=ExternalPose(lever,('x','y','z','rx','ry','rz')).residual(x,np.asarray(r['ndt']['final']));U=np.asarray(r['ndt'].get('av_projection',{}).get('matrix',np.eye(6)));z=scale_mixture_moments(np.asarray(r['prior']['P']),U@H,np.asarray(u['R']),U@res,256);B=chart_map(np.asarray(u['delta']));Qerr=max(Qerr,float(np.max(abs(B@z['covariance']@B.T-np.asarray(u['P_after_native'])))));meanerr=max(meanerr,float(np.max(abs(z['mean']-u['delta']))));selected+=1
 assert delta_error<1e-12 and meanerr<1e-9 and Qerr<1e-9
 a.output.write_text(json.dumps(dict(frozen_sha256=checks,preexisting_source_and_reports_identical=len(files),legacy_actual_CUDA_reproduction=reproduction,actual_mixture_frames=len(tx),all_delta_Kr_max_error=delta_error,order256_crosscheck_selected_frames=selected,mean_max_difference=meanerr,full_postreset_covariance_max_difference=Qerr,statistical_calibration=False),indent=2)+'\n')
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True);main(p.parse_args())
