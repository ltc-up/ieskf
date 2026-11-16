"""Validate saved bounded results; never launches registration/ROS/estimator."""
import json,sys
from pathlib import Path
import numpy as np

def main(root):
 r=Path(root);checks={};matches=[json.loads(s) for s in open(r/'frozen/results.jsonl')];checks['registered_35_unique_within40']=len(matches)==35 and len(set(x['request']['case_id'] for x in matches))==35
 checks['input_point_count_preserved']=all(x['result']['source_n']==x['request']['identity']['source_n'] for x in matches)
 checks['config_q50_max35']=all(x['request']['q']==50 and x['request']['max_iterations']==35 for x in matches)
 checks['all_native_R_finite_symmetric_SPD']=all(np.isfinite(x['result']['native_internal']).all() and np.allclose(x['result']['native_internal'],np.array(x['result']['native_internal']).T,atol=1e-10) and np.linalg.eigvalsh(x['result']['native_internal']).min()>0 for x in matches)
 checks['all_full_gradient_H_finite']=all(np.isfinite(x['result']['score_hessian']).all() and np.isfinite(x['result']['score_gradient']).all() for x in matches)
 checks['failure_count_7_preserved']=sum(not x['result']['converged'] for x in matches)==7
 x=json.load(open(r/'analysis/native_probe.json'));checks['native_trace_mean_P_exact']=x['trace_equivalence_max_abs']==0;checks['same_50ms_comparison']=x['comparison_horizon_ms']==50
 checks['native_repeated_same_id_stronger_information']=x['repeat5_Pyy']<x['single_Pyy'] and x['repeat5_position_y']>x['single_update_position_y']
 checks['native_trace_P_finite_symmetric_PSD']=all(np.isfinite(q['Ppost']).all() and np.max(abs(np.array(q['Ppost'])-np.array(q['Ppost']).T))<1e-10 and np.linalg.eigvalsh(q['Ppost']).min()>-1e-10 for q in x['same_id_repeated5'])
 z=json.load(open(r/'prototype_v2/ros_results.json'));checks['ROS_401_unique_received']=z['published']==z['received']==z['unique_received']==z['expected_slots']==401 and z['skipped']==0 and not z['missing_received'];checks['ROS_no_state_time_reversal_or_errors']=not z['time_reversal'] and not z['errors'];checks['ROS_numerical_hash_parity']=z['event_hash_equal'] and z['final_state_P_hash_equal'];checks['planned_stale_outputs_retained']=z['stale']==19
 checks['compiled_native_headers_equal_official']=all(q['equal'] for q in json.load(open(r/'analysis/native_header_identity.json')))
 checks['three_map_additions_exact_superset']=all(q['base_missing_from_augmented']==0 and q['added_unique']==4621 for q in json.load(open(r/'analysis/map_addition_identity.json')))
 z=json.load(open(r/'analysis/native_jacobian.json'));checks['native_zero_tilt_derivative_mismatch_detected']=z['source_zero_block_max_difference']>.09
 z=json.load(open(r/'analysis/far_recovery_aggregate.json'));checks['qualified_recovery_356_not_floor_imputation']=z['qualified']==356 and z['floor_ineligible']==724 and z['strict_10um_pass']==158
 out=dict(scope='saved result contract checks, not localization accuracy/statistical calibration qualification',checks=checks,passed=sum(checks.values()),total=len(checks));(r/'analysis/validation_results.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2));assert all(checks.values())
if __name__=='__main__':main(sys.argv[1])
