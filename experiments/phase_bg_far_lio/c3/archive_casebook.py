"""Read only already archived historical evidence; no new registration or filter."""
import csv,hashlib,json,sys
from pathlib import Path
import numpy as np

def main(out,c1,be,archive1,archive2):
 out=Path(out);a1=Path(archive1);a2=Path(archive2);source=a2/'seed_robustness.csv';sweep=list(csv.DictReader(source.open()));ident=list(csv.DictReader((a2/'same_input_sha256.csv').open()));keep=[x for x in sweep if 4<=float(x['xy_error'])<=5];hist=[]
 for x in keep:
  z={k:x[k] for k in ['case','kind','lateral_delta','longitudinal_delta','yaw_delta_deg','iterations','converged','termination','objective_score','pcl_fitness','xy_error','longitudinal_error','lateral_error','yaw_error_deg','basin']};z.update(source_row_sha256=hashlib.sha256(json.dumps(x,sort_keys=True).encode()).hexdigest(),prior_P_K_R='MISSING; no substitute prior and no R-update computation',label_evidence='original archived WRONG_BASIN plus original engineering reference; not independent truth',input_identity=next((r for r in ident if r['case']==x['case']),None));hist.append(z)
 correct_large=[x for x in sweep if x['basin']=='CORRECT_BASIN' and abs(float(x['corr_body_y']))>=.3];all_counts={case:{label:sum(x['case']==case and x['basin']==label for x in sweep) for label in sorted(set(x['basin'] for x in sweep))} for case in sorted(set(x['case'] for x in sweep))}
 gt=list(csv.DictReader((a1/'gnss_truth_comparison.csv').open()));one=next(x for x in gt if x['frame_id']=='12632');audit=list(csv.DictReader((a1/'baseline_mt_feature_audit.csv').open()));onea=next(x for x in audit if x['frame_id']=='12632')
 z=dict(scope='independent archived matcher cases, not pooled into C1 R statistics',history_12632={k:one[k] for k in ['frame_id','cpu_gnss_xy_error','cuda_gnss_xy_error','cpu_gnss_longitudinal_error','cuda_gnss_longitudinal_error','cpu_gnss_lateral_error','cuda_gnss_lateral_error']},history_12632_optimizer={k:onea[k] for k in ['final_basin','iterations','final_score','mean_mt_eval_count','step_min_clamp_total']},history_4_to_5m_cases=hist,original21grid_counts=all_counts,correct_large_original_proxy_count=len(correct_large),missing=['same-time 15D prior/P/K/R for archived4–5m/12632','independent truth certifying basin labels','fully qualified old GNSS0/75/150 contract matching current C1'],identities={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in [source,a2/'same_input_sha256.csv',a1/'gnss_truth_comparison.csv',a1/'baseline_mt_feature_audit.csv']})
 # Retain full old selected rows privately, never maps/clouds or public coordinates.
 (out/'historical_selected_rows.json').write_text(json.dumps(dict(four_to_five=keep,correct_large_proxy=correct_large),indent=2));(out/'historical_casebook_aggregate.json').write_text(json.dumps(z,indent=2))
 # Three subsequent observed candidates are evidence only, not online decisions.
 ref=np.load(Path(be)/'runtime/reference_lap.npz');lookup={int(t):i for i,t in enumerate(ref['header'])};seq=[json.loads(l) for l in (Path(c1)/'eskf15_recorded_init/matches.jsonl').open()];seq=[r for r in seq if 'ndt' in r];pos={r['stamp_ns']:i for i,r in enumerate(seq)};ledger=json.load(open(out/'casebook_ledger.json'));count=0
 for case in ledger:
  if case['dataset']!='C1':case['followup']=dict(status='existing BE original records retained; no causal-confirmation analysis');continue
  i=pos[case['stamp']];case['followup']=[]
  for r in seq[i+1:i+4]:
   case['followup'].append(dict(index=lookup.get(r['stamp_ns']),header=r['stamp_ns'],available=r['available_ns'],complete=r['complete_sim_ns'],converged=r['ndt']['converged'],source_n=r['ndt']['source_n'],inliers=r['ndt']['inliers_at_float_pose'],fitness=r['ndt']['fitness'],accepted=r['update']['accepted'],decision_use='offline evidence only; unavailable before its completion'))
  count+=1
 (out/'casebook_with_followup.json').write_text(json.dumps(ledger,indent=2));print(json.dumps(dict(archived4to5=len(keep),converged_wrong_cuda=sum(x['converged']=='1' and x['case'].startswith('cuda') for x in keep),correct_large_proxy=len(correct_large),C1_case_followups=count)))
if __name__=='__main__':main(*sys.argv[1:])
