"""Seal non-reconstructive identities and aggregate runtime; no private payload copy."""
import argparse,json,hashlib,subprocess
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--private-root',type=Path,required=True);a=p.parse_args();root=Path(__file__).parent
sha=lambda p:hashlib.file_digest(Path(p).open('rb'),'sha256').hexdigest()
inputs={'motion':'data/phase_ao_ap_validation/loop44.json','target':'data/phase_ar_loop44_map0105/map0105_target.bin','config':'data/history/phase_v/config/frozen.json','AR_transactions':'data/phase_ar_loop44_map0105/fullpose100_lap/transactions.json','AS_transactions':'data/phase_as_accuracy_quality/F_multilap/transactions.json','AW_transactions':'data/phase_aw_native_gicp_fusion/private/gicp_lap/transactions.json','AT_causal':'data/phase_at_gnss_footprint_corners/causal.npz','AR_bindings':'data/phase_ar_loop44_map0105/bindingslap.json','AS_bindings':'data/phase_as_accuracy_quality/bindings_multilap.json','reference_cache':'data/phase_aw_native_gicp_fusion/private/gicp_multilap/evaluation/evaluation.npz'}
expected={'motion':'7445c7172792d93352d876c3030acdef498ae9d68e21fa6c69a94bf68852719c','target':'0895ad26af296aeba68b55350f7b9b70cd6b397be551fc78af75794410192968','config':'c595a50d62c841c8c6379a42c37bf6afc79592985daef8db77fbdff5c43da29f','AR_transactions':'cac59aa187bec29f0e23039ff8a28e9414ad30939600ede811dae3612a21c7fc','AS_transactions':'dfd6e4db1ef8b6aa2124b761f7601afb77edebd85fa9dab8963130bfe582f7c1'}
ids={k:sha(v) for k,v in inputs.items()}
for k,v in expected.items():assert ids[k]==v,('HISTORICAL_IDENTITY_CHANGED',k)
bag=(a.private_root/'actual_bag_sha256.txt').read_text().split()[0];assert bag=='0be29d090557accc27581007b45660cb5b0063dbcbdd3f3b798e33ff64c1db14'
# All pre-existing estimator modules must remain byte-for-byte frozen.
tracked=subprocess.check_output(['git','ls-tree','-r','--name-only','c647ac3','src'],text=True).splitlines();checked=0
for path in tracked:
 data=subprocess.check_output(['git','show','c647ac3:'+path]);assert Path(path).read_bytes()==data,path;checked+=1
code=[*root.glob('*.py'),*root.glob('*.cpp'),*root.glob('*.sh'),Path('src/geometry_ieskf/direct_iterated.py'),Path('tests/test_ax_direct.py')]
private={}
for name in ['A40_v1','A40_v2','A40_v3','A40_v4','A_lap','A_multilap','A_instrumented_lap','B40','B40_rawplanes','B_lap','legacy40']:
 private[name]={k:sha(a.private_root/name/k) for k in ['transactions.json','states.json']}
binaries={k:sha(a.private_root/'runtime'/k) for k in ['direct_service','direct_service_v3','direct_service_rawplanes','export_gaussians']}
manifest=dict(phase='AX',frozen_main='c647ac3224adee775d30dc6f9553c9b83612c060',implementation_commit=subprocess.check_output(['git','rev-parse','aa25f12'],text=True).strip(),containing_results_commit='THIS_COMMIT; do not self-hash',AW_reuse='e15e7a235a096dbf67dcf154b5d88c8ec3e6e4e4',small_gicp_upstream='fa0cfc983417c044b6036f71dfa448038bb9c820',production_head='aa50e35467f89b4963f882d1f5abaace46d0ce70',actual_bag_sha256=bag,private_input_sha256=ids,original_estimator_files_unchanged=checked,code_sha256={str(p.resolve().relative_to(Path.cwd())):sha(p) for p in code},binary_sha256=binaries,private_artifact_sha256=private,gaussian_export_sha256=sha(a.private_root/'map0105_gaussians.bin'),tests=dict(passed=205,failed=0,skipped=0,new_AX=10,reused_AW=12,frozen=183),actual_data=dict(A_lap_frames=1468,A_lap_updates=1468,A_multilap_frames=4134,A_multilap_updates=4134,B_lap_requested=1468,B_lap_processed=531,B_lap_updates=451,B_three_laps='NOT_EXECUTED'),qualifications=dict(A_function='VERIFIED_CONDITIONAL_DIRECT_MAP',A_accuracy_replacement='FAILED',B_lap='FAILED',statistical_consistency='NOT_VALIDATED',certified_reference='NOT_VALIDATED',ROS20Hz='NOT_VALIDATED',production='NO_GO',main_merge='NOT_AUTHORIZED_ON_FAILED_ACCEPTANCE'),privacy='Only code, synthetic controls, aggregate statistics and deidentified plots; private full payloads remain external')
(root/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
# Aggregate timing from already executed metadata; no new state replay.
rows=[json.loads(s) for s in (a.private_root/'A_instrumented_lap/transactions.jsonl').open()];ds=[json.loads(s) for s in (a.private_root/'A_instrumented_lap/deskew_meta_stream.jsonl').open()]
import sys;sys.path.insert(0,str(root.parent/'phase_aw_native_gicp_fusion'));from evaluate import stats
r={k:stats([s[k] for s in ds]) for k in ['gpu_deskew_ms','crop_voxel_sample_ms','rebase_with_file_write_ms']};r['direct_MAP_observer_wall_ms']=stats([s['update']['wall_ms'] for s in rows]);r['time_semantics']='Per-module quantiles; no summation into end-to-endP95. Direct observer excludes suffix replay; full section covers it. Separate per-call IMU/wheel/suffix time NOT_INSTRUMENTED in AX.';(root/'module_timing.json').write_text(json.dumps(r,indent=2)+'\n')
print(json.dumps(dict(sealed_inputs=len(ids),original_modules=checked,actual_bag_sha256=bag,tests=205)))
