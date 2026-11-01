import sys,json,numpy as np
from pathlib import Path
from geometry_ieskf.common import dump
from geometry_ieskf.objective import state
from geometry_ieskf.geometry import GeometryFactor
import argparse,os
p=argparse.ArgumentParser();p.add_argument('--data-root',default=os.environ.get('IESKF_DATA_ROOT'));p.add_argument('--public-output',required=True);args=p.parse_args();assert args.data_root;R=Path(args.data_root);D=R/'phase_aj_ndt_pose_update';E=Path(args.public_output);c=json.load(open(D/'C_replay_private.json'));a=json.load(open(R/'history/phase_v/results/replay_inertial_wheel_control_continuous_extension.json'));l=c['lidar_updates'];transactions=[json.loads(s) for s in open(D/'ndt_transactions.jsonl')];factor=GeometryFactor(json.load(open(R/'history/phase_v/results/input_contract.json'))['extrinsic']['lever_body_imu_m']);assert len(transactions)==284
seed_error=0.;source_binding=[]
for i,(r,t) in enumerate(zip(l,transactions)):
 expected=factor.transform(state(r['predicted']));seed_error=max(seed_error,float(abs(expected-t['request']['seed_common']).max()));assert np.array_equal(np.array(t['measurement']['seed_float']),expected.astype(np.float32).astype(float));source_binding.append(t['update']['source_float32_sha256'])
assert seed_error<2e-12,seed_error
first=next(i for i,r in enumerate(l) if r['accepted']);pre_errors={k:max(float(abs(np.array(r['predicted'][k])-a['lidar_updates'][i]['predicted'][k]).max()) for i,r in enumerate(l[:first+1])) for k in ['p_imu_map','v_imu_map','q_map_body_xyzw','bg_body_rad_s','ba_body_m_s2','P']};assert max(pre_errors.values())==0
allP=np.array([s['P'] for s in c['imu_states']]+[s['corrected']['P'] for s in l]+[s['predicted']['P'] for s in l]);min_eig=float(np.linalg.eigvalsh(allP).min());assert min_eig>0
maxactual=max([(i,r) for i,r in enumerate(l) if r['accepted']],key=lambda a:abs(a[1]['delta'][5]));maxproposed=max([(i,r) for i,r in enumerate(l) if 'delta_proposed' in r],key=lambda a:abs(a[1]['delta_proposed'][5]));examples={}
for name,(i,r) in [('largest_actual_vz',maxactual),('largest_proposed_vz',maxproposed)]:
 components=np.array(r['K'])[5]*np.array(r['innovation']);examples[name]=dict(event_alias=f'L{i:03d}',accepted=r['accepted'],reason=r['reason'],proposed_delta_Z_m=r['delta_proposed'][2],proposed_delta_vz_m_s=r['delta_proposed'][5],actual_delta_vz_m_s=r['delta'][5] if r['accepted'] else 0.,signed_vz_contribution_XY_yaw_m_s=components.tolist(),guard_failures=r['guard_failures'])
extra=dict(causal_seed_from_C_prediction=True,seed_snapshot_roundtrip_max_abs_error=seed_error,all_seed_float32_casts_exact=True,pre_first_accept_C_A_state_P_max_errors={name+'_max_abs_error':value for name,value in zip(['position','velocity','orientation','gyro_bias','accel_bias','covariance'],pre_errors.values())},all_recorded_IMU_and_lidar_P_min_eigenvalue=min_eig,signed_update_examples=examples,accepted_with_fitness_at_least_3=sum(r['accepted'] and r['ndt']['fitness']>=3 for r in l),performance_slow_event_aliases=[f'L{i:03d}' for i,r in enumerate(l) if r['wall_ms']>50],initial_lidar_rejected_atomic_P_equal=bool(np.array_equal(l[0]['predicted']['P'],l[0]['corrected']['P'])),max_abs_source_float_cast_error_bound_note='Float32 backend conversion disclosed; same points, source order and reference, not scalar-bit identity to float64 geometry branch')
q=json.load(open(E/'key_results.json'));q['extra_audit']=extra;dump(E/'key_results.json',q);dump(D/'extra_audit_summary.json',extra);dump(D/'backend_source_hashes_private.json',source_binding)
# Coarse non-reconstructive scalar bins only: no XY/map coordinates or sample trajectory.
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
bins=json.load(open(E/'height_binned_summary.json'));fig,axes=plt.subplots(3,1,figsize=(9,8),sharex=True)
for k,rows in bins.items():
 ts=[.5*sum(r['elapsed_bin_s']) for r in rows]
 for ax,key,band in [(axes[0],'relative_Z_m_median','relative_Z_m_P10_P90'),(axes[1],'vz_m_s_median','vz_m_s_P10_P90')]:
  ax.plot(ts,[r[key] for r in rows],label=k);ax.fill_between(ts,[r[band][0] for r in rows],[r[band][1] for r in rows],alpha=.12)
 axes[2].plot(ts,[r['sigma_Z_m_median'] for r in rows],label=k)
axes[0].set_ylabel('Relative Z (m)');axes[1].set_ylabel('vz (m/s)');axes[2].set_ylabel('sigma Z (m)');axes[2].set_xlabel('Elapsed seconds (1 s aggregate bins)')
for ax in axes:ax.grid(alpha=.2);ax.legend()
fig.suptitle('AJ: 1-second median / P10-P90 summaries; no independent truth');fig.tight_layout();fig.savefig(E/'height_summary.png',dpi=150);plt.close(fig)
print(extra)
