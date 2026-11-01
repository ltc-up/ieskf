"""Frozen observation validation; raw inputs/results remain local-only."""
import argparse,os,json,numpy as np
from pathlib import Path
from geometry_ieskf.common import dump,sha
from geometry_ieskf.objective import state
from geometry_ieskf.state import difference,inject
from geometry_ieskf.ndt_pose_observation import NdtPoseObservation,engineering_R,pose_xyyaw,wrap

def main(root):
 root=Path(root);out=root/'phase_aj_ndt_pose_update';ah=root/'phase_ah_deskew_z_ablation';selection=json.load(open(ah/'selection_private.json'))['selected'];old=json.load(open(ah/'frozen_updates_private.json'));ndts=[json.loads(l) for l in open(ah/'ndt_results.jsonl')];contract=json.load(open(root/'history/phase_v/results/input_contract.json'));obs=NdtPoseObservation(contract['extrinsic']['lever_body_imu_m']);private=[];public=[]
 for e in selection:
  rec=next(x for x in old if x['alias']==e['alias'] and x['group']=='A');ndt=next(x for x in ndts if x['case_id']==e['alias']+'_A');prior=state(rec['prior']);P=np.array(rec['prior']['P']);assert prior.t==e['frame']['header_ns'];T=np.array(ndt['final']);z=pose_xyyaw(T);R=engineering_R(ndt['fitness']);x,Q,info=obs.correct(prior,P,z,R);h,H=obs.model(prior);fd=np.zeros_like(H)
  for k in range(15):
   d=np.eye(15)[k]*1e-6;a=obs.model(inject(prior,d))[0];b=obs.model(inject(prior,-d))[0];v=a-b;v[2]=wrap(v[2]);fd[:,k]=v/2e-6
  err=float(np.max(abs(fd-H)));assert err<1e-6
  if not info['accepted']:np.testing.assert_array_equal(Q,P);np.testing.assert_array_equal(difference(x,prior),np.zeros(15))
  else:assert np.linalg.eigvalsh(Q).min()>0 and info['covariance_updates']==1
  private.append(dict(alias=e['alias'],prior=rec['prior'],T_ndt=T,measurement=z,noise=R,H=H,info=info,actual_delta=difference(x,prior),P_post=Q));public.append(dict(alias=e['alias'],jacobian_FD_max_error=err,accepted=info['accepted'],reason=info['reason'],guard_failures=info.get('guard_failures',[]),actual_delta_Z_m=float(difference(x,prior)[2]),actual_delta_vz_m_s=float(difference(x,prior)[5]),P_post_positive=bool(np.linalg.eigvalsh(Q).min()>0)))
 dump(out/'frozen_private.json',private);dump(out/'frozen_summary.json',public);print(public)
if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('--data-root',default=os.environ.get('IESKF_DATA_ROOT'));p=a.parse_args();assert p.data_root;main(p.data_root)
