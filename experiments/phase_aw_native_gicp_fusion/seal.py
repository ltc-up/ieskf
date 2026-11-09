"""Seal non-reconstructive identities, effective models and actual execution counts."""
import argparse,json,hashlib,subprocess
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--private-root',type=Path,required=True);p.add_argument('--production-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();D=Path(__file__).parent;BASE='c647ac3224adee775d30dc6f9553c9b83612c060'
def sha(path):
 h=hashlib.sha256()
 with Path(path).open('rb') as f:
  for b in iter(lambda:f.read(8*1024*1024),b''):h.update(b)
 return h.hexdigest()
original=json.loads((D/'integrity.json').read_text());files=subprocess.check_output(['git','ls-tree','-r','--name-only',BASE,'--','src','experiments/phase_ar_loop44_map0105','experiments/phase_as_accuracy_quality','experiments/phase_at_gnss_footprint_corners','experiments/phase_au_prediction_feedback'],text=True).splitlines()
for f in files:assert Path(f).read_bytes()==subprocess.check_output(['git','show',BASE+':'+f]),f
runs={}
for name in ['legacy40','gicp_lap','gicp_multilap','ndt_fixed_lap','gicp_profile200','gicp_profile_lap']:
 path=a.private_root/name;tx=json.loads((path/'transactions.json').read_text());isg=name.startswith('gicp');accepted=[r for r in tx if r['update']['accepted']]
 if isg:assert all(r['ndt'].get('backend')=='small_gicp' for r in tx)
 if isg or name=='ndt_fixed_lap':
  expected=np.diag([.25]*3+[np.deg2rad(1.)**2]*3)
  for row in accepted:np.testing.assert_allclose(row['update']['R'],expected,atol=1e-16)
 runs[name]=dict(frames=len(tx),duration_s=(tx[-1]['ref']-tx[0]['ref'])*1e-9,updates=len(accepted),matcher='small_gicp_cpu' if isg else 'cuda_ndt',CUDA_Deskew=True,own_state_matching=True,transactions_sha256=sha(path/'transactions.json'),states_sha256=sha(path/'states.json'),summary_sha256=sha(a.private_root/(name+'_summary.json')),history_horizon_ms=500)
 if name=='gicp_multilap':runs[name]['preselected_lap_updates']=[sum(row['update']['accepted'] for row in tx[lo:hi]) for lo,hi in [(0,1468),(1468,2820),(2820,4134)]]
source_head=subprocess.check_output(['git','-C',str(a.production_root),'rev-parse','HEAD'],text=True).strip();assert source_head=='aa50e35467f89b4963f882d1f5abaace46d0ce70';stash=subprocess.check_output(['git','-C',str(a.production_root),'stash','list','--format=%H'],text=True).splitlines();expected_stashes=['c964ce45adc5298a473f2d8387ee7a53ba5ec516','c4ecdcf7f1c9688095e0e1e2b2a46bd9da1b1453','65505cefdb33b7e75d571077628ef2bc20de6635'];assert all(x in stash for x in expected_stashes)
code=[Path('src/ndt_global/matcher_geometry.py'),*D.glob('*.py'),D/'gicp_service.cpp',D/'build_gicp.sh'];artifact=(a.private_root/'matcher_ab_fixed/matches.jsonl');bad=a.private_root/'matcher_ab/matches.jsonl';result=dict(phase='AW1_AW2_AW3_AW4',baseline=BASE,final_reporting_commit='enclosing commit',actual_implementation_commit='042d00a',AW1_commit='2f0c50a',source_recovery=dict(production_HEAD=source_head,production_branch='ndt_test',preserved_stashes=expected_stashes,stash0_untracked_tree_empty=True,third_party_revision='fa0cfc983417c044b6036f71dfa448038bb9c820',license='MIT',production_modified=False),configuration=dict(GICP_threads_TBB=4,kNN=20,max_iterations=50,translation_epsilon_m=.001,rotation_epsilon_rad=.0017453292519943296,maximum_correspondence_m=4.,extra_source_target_voxel=False,fixed_observation_std_m=.5,fixed_observation_std_rotation_deg=1.,common_geometry_rotation_length_m=10.,original_state_Q_wheel_bias_reset_unchanged=True,GNSS_fused=False,AV_used_as_default=False),tests=dict(baseline=183,new=12,passed=195,failed=0,skipped_final=0,executable_tests=3,log_sha256=sha(a.private_root/'tests_final195.log')),input_identity=original,actual_runs=runs,same_input_AB=dict(frames=1468,ndt_valid_converged=1468,gicp_valid_converged=1399,matches_sha256=sha(artifact),probes_sha256=sha(a.private_root/'matcher_ab_fixed/probes.jsonl')),preserved_failed_adapter=dict(frames=1468,valid_GICP=0,matches_sha256=sha(bad),snapshot_sha256=sha(a.private_root/'source_recovery/adapter_dangling_expression_snapshot.cpp'),original_binary_observed_sha256='deca16ae88bb29421314d26dd50e19a45626445a658a72528fa30adbf973d9d3',old_binary_not_retained_but_source_and_logs_preserved=True),backend_binary_sha256=sha(a.private_root/'runtime/gicp_service'),source_code_sha256={str(f):sha(f) for f in code},reference_cache_sha256=sha(a.private_root/'gicp_multilap/evaluation/evaluation.npz'),GNSS_original_CSV_currently_missing=True,preexisting_source_report_files_unchanged=len(files),qualification=dict(source_numeric_function='VERIFIED',single_and_three_lap_execution='VERIFIED',engineering_reference='CONDITIONAL',replacement_accuracy='FAILED',adaptive_directional_R='NOT_IMPLEMENTED_INSUFFICIENT_EVIDENCE',covariance_calibration='NOT_VALIDATED',strict_offline_50ms_deadline='FAILED',ROS20Hz100Hz='NOT_VALIDATED',production='NO_GO'),main_merge=False,branch='research/phase-aw-native-gicp',private_payloads_committed=False)
at_manifest=json.loads(Path('experiments/phase_at_gnss_footprint_corners/manifest.json').read_text());at_binding=json.loads(Path('data/phase_at_gnss_footprint_corners/bindings.json').read_text());reference_inputs={}
for key,expected in at_manifest['input_sha256'].items():
 path=Path(at_binding[key]);exists=path.exists();actual=sha(path) if exists else None
 if exists:assert actual==expected,'AT_REFERENCE_INPUT_CHANGED:'+key
 reference_inputs[key]=dict(exists=exists,expected_sha256=expected,actual_sha256=actual)
result['reference_input_identity']=reference_inputs;result['reference_bound_sha256']=sha(Path(at_binding['private'])/'reference_bound.json');result['private_visual_outline_sha256']=sha(Path('data/phase_at_gnss_footprint_corners/visual_outline.json'));result['runtime_binary_sha256']={key:sha(Path('data/phase_ar_loop44_map0105/runtime')/name) for key,name in [('CUDA_NDT','match_fixed100'),('CUDA_Deskew','deskew_service'),('CUDA_NDT_library','build_fixed/libndt_gpu_matcher.so')]}
result['compiler_version']=subprocess.check_output(['c++','--version'],text=True).splitlines()[0]
a.output.write_text(json.dumps(result,indent=2)+'\n')
