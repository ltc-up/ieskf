"""Publish only aggregate replay evidence; private raw trajectories never copied."""
import argparse,json,hashlib,collections
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();out=a.out;out.mkdir(exist_ok=True);runs={}
for window in ['lap','multilap']:
 for mode in ['fullbaseline','matched','directional','optical','dual','gmf','robust']:
  choices=[a.root/'results'/f'{prefix}_{mode}_{window}' for prefix in ['controlled2','controlled','revised','final']]
  valid=[r for r in choices if (r/'summary.json').exists() and (r/'metrics.json').exists()]
  if not valid:continue
  r=valid[0];m=json.load(open(r/'metrics.json'));s=json.load(open(r/'summary.json'));versions=json.load(open(r/'source_version.json'));m['source_hashes']={Path(k).name:v for k,v in versions.items()};m['run']=r.name;m['decisions']=s['decisions'];m['weight_revisions']=s.get('weight_revisions',[]);m['startup_cache_warm_ms']=s['startup_ms'];m['configuration']={k:v for k,v in s['configuration'].items() if k not in ['paths','out']};m['switch_events']=[{k:v for k,v in event.items() if k!='stamp_ns'} for event in s['switches']];runs[f'{mode}_{window}']=m
(out/'aggregate_results.json').write_text(json.dumps(runs,indent=2))
rows=['# Accuracy comparison','', 'All numbers are engineering-reference differences, not certified truth. GNSS is offline only, with historical antenna-to-base conversion. The primary table evaluates reconstructed scan-time posteriors; `aggregate_results.json` separately evaluates the delivered current state at its actual timestamp (interpolated reference, no extrapolation). Empty/out-of-span samples are explicit. Proxy-good/bad labels do not establish true match correctness.','', 'Full-source/100-iteration original15D control uses explicit legacy lifecycle; the held-out/35-iteration matched control uses the managed lifecycle. Both preserve the original motion/pose models. These execution controls are not asserted bitwise equivalent to historical timing. New methods must be compared to matched control. All use the reused GLL3 research matcher. The audited aa50 lidar_ndt source/YAML also selects GLL3; historical c647 OFF and an unverified deployed lidar_ndt_v1 path must be distinguished. Source/binary and deployed wiring equivalence are not asserted. Production six-state performance is not measured here.','', '| Run | Frames | XY RMSE 0/75/150ms (m) | Lateral P50/P95/max 75ms (m) | Longitudinal RMSE (m) | Yaw RMSE (deg) |','|---|---:|---|---|---:|---:|']
for name,m in runs.items():
 q=m['sensitivity']['0.075'];v=[m['sensitivity'][t]['xy'].get('rmse',float('nan')) for t in ['0.0','0.075','0.15']];l=q['lateral'];rows.append(f"| {name} | {m['frames']} | {' / '.join(f'{x:.3f}' for x in v)} | {l.get('p50',0):.3f} / {l.get('p95',0):.3f} / {l.get('max',0):.3f} | {q['longitudinal'].get('rmse',0):.3f} | {q['yaw_deg'].get('rmse',0):.3f} |")
rows+=['','## Fixed sections — lateral P95 at 75ms','', '| Run | Right 3 | Right 9 | Normal 5 | Normal 8 |','|---|---:|---:|---:|---:|']
for name,m in runs.items():
 seg=m['sensitivity']['0.075']['segments'];rows.append('| '+name+' | '+' | '.join(f"{seg[str(i)]['lateral'].get('p95',float('nan')):.3f}" for i in [3,9,5,8])+' |')
rows+=['','All ten sections, longitudinal distributions, yaw, validity masks/counts, proxy admission counts, branch counts, switches and both timestamp protocols are in the aggregate JSON. Admission labels use frozen 0.5m/2m proxy cutoffs with an explicit ambiguous band. They are not a new online gate or reliable correctness-rate estimate. Mode recognition latency against a true mode is UNKNOWN because independent mode labels are unavailable.','', '## Failure conditions','', 'A scheme fails qualification if normal sections regress, timing/reference assumptions reverse its ranking, modes cannot be independently identified, expired data dominate, or inclusive runtime exceeds the budget. A smaller global RMSE alone is insufficient. The full failure inventory is retained alongside complete runs.']
(out/'ACCURACY_COMPARISON.md').write_text('\n'.join(rows)+'\n')
rows=['# Inclusive performance comparison','', 'Serial configurations; real CPU small_gicp and CUDA-NDT/deskew. Within a first-frame candidate request, CPU/GPU backends may overlap; two established branches have independent deskew and are subsequently processed in sequence. Reported component times overlap in the first case and must not be summed as wall time. Inclusive cycle includes file IO and logging. No CUDA GICP was used. The startup field is service cache-warm time, not all process/validator initialization.','', '| Run | P50 | P95 | P99 | max | >50ms |','|---|---:|---:|---:|---:|---:|']
for name,m in runs.items():
 t=m['cycle_ms'];rows.append(f"| {name} | {t['p50']:.2f} | {t['p95']:.2f} | {t['p99']:.2f} | {t['max']:.2f} | {m['over_50ms']} |")
rows+=['','All values are milliseconds. Complete CUDA-NDT, GICP service, deskew, geometric validation, delayed-update/repropagation, GMF and rollback distributions are in `aggregate_results.json`. Matching service wall includes its neighborhood/derivative/optimizer work; these binaries do not expose an independently separable neighborhood/Hessian/line-search timer for every path. Such missing decomposition is UNKNOWN, never reported as zero. GPU memory and CPU/GPU contention were not sampled continuously, so no resource-isolation guarantee is claimed.','', 'Strict 50ms qualification requires every complete cycle, including recovery, to fit. A good P95 does not satisfy that condition. Source-history archival allows offline processing of all raw scans despite backlog; it does not make late observations timely. Measurement history stays 500ms and expired observations do not enter the filter. Cold failures, user pause/interruption and pre-fix slow paths remain in the private logs and public failure inventory.']
(out/'PERFORMANCE_COMPARISON.md').write_text('\n'.join(rows)+'\n')
# De-identified failure inventory, no traceback paths or original payload.
fail=[]
for r in sorted((a.root/'results').iterdir()):
 if not r.is_dir() or not (r/'frames.jsonl').exists() or (r/'summary.json').exists():continue
 with (r/'frames.jsonl').open() as f:n=sum(1 for _ in f)
 log=a.root/'logs'/(r.name+'.log');reason='no final summary'
 if log.exists():
  lines=log.read_text(errors='replace').splitlines()
  for line in reversed(lines):
   if line.startswith(('ValueError:','AssertionError','RuntimeError:','KeyboardInterrupt')):reason=line[:200];break
 fail.append(dict(run=r.name,completed_prefix=n,reason=reason))
for log in sorted((a.root/'logs').glob('*.log')):
 if (a.root/'results'/log.stem/'frames.jsonl').exists():continue
 lines=log.read_text(errors='replace').splitlines()
 reasons=[line for line in lines if line.startswith('IndentationError:')]
 if reasons:fail.append(dict(run=log.stem,completed_prefix=0,reason=reasons[-1],classification='startup compile failure, no scans processed'))
(out/'failure_inventory.json').write_text(json.dumps(fail,indent=2))
partial={}
for f in fail:
 r=a.root/'results'/f['run']
 if (r/'metrics.json').exists():partial[f['run']]=json.load(open(r/'metrics.json'))
(out/'partial_results.json').write_text(json.dumps(partial,indent=2))
for name in ['diagnostics','recovery_analysis']:
 file=a.root/'results'/(name+'.json')
 if file.exists():(out/('diagnostic_summary.json' if name=='diagnostics' else name+'.json')).write_text(file.read_text())
file=a.root/'audit/frozen_validation_review/frozen_summary.json'
if not file.exists():file=a.root/'results/e2_frozen_lap/frozen_summary.json'
if file.exists():(out/'frozen_summary.json').write_text(file.read_text())
file=a.root/'results/lidar_information.json'
if file.exists():
 data=json.load(open(file));parts=collections.defaultdict(list)
 for row in data:
  for key,v in row['parts'].items():parts[key].append(np.asarray(v['information']))
 summary={'sampled_frames':len(data),'all_sum_checks':all(v['sum_exact'] for v in data),'parts':{key:{'frames':len(v),'mean_information_trace':float(np.mean([np.trace(L) for L in v])),'mean_eigenvalues':np.mean([np.linalg.eigvalsh(L) for L in v],axis=0).tolist()} for key,v in parts.items()},'interpretation':'sampled pre-voxel common-map diagnostic; additive does not imply independent statistical information'}
 (out/'lidar_summary.json').write_text(json.dumps(summary,indent=2))
file=a.root/'audit/directional_current_review.json'
if file.exists():(out/'directional_current_review.json').write_text(file.read_text())
# Privacy-safe aggregate chart.
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,axes=plt.subplots(1,2,figsize=(12,5))
for window,color in [('lap','tab:blue'),('multilap','tab:orange')]:
 names=[k for k in runs if k.endswith('_'+window)];x=np.arange(len(names))+(0 if window=='lap' else .35)
 axes[0].bar(x,[runs[k]['sensitivity']['0.075']['xy'].get('rmse',np.nan) for k in names],width=.35,color=color,label=window)
 axes[1].bar(x,[runs[k]['cycle_ms']['p95'] for k in names],width=.35,color=color,label=window)
labels=['fullbaseline','matched','directional','optical','dual','gmf','robust']
for ax in axes:ax.set_xticks(np.arange(7)+.175,labels,rotation=40,ha='right');ax.legend();ax.grid(axis='y',alpha=.2)
axes[0].set_ylabel('XY reference RMSE (m), 75ms');axes[1].set_ylabel('Inclusive cycle P95 (ms)');axes[1].axhline(50,color='red',ls='--');fig.tight_layout();fig.savefig(out/'aggregate_comparison.png',dpi=160);plt.close(fig)
print('aggregated',len(runs),'complete comparisons;',len(fail),'failed prefixes')
