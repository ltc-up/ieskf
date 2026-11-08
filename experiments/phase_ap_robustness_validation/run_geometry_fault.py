"""Actual frozen Source/Target geometry + a predetermined artificial map hole.
No re-registration, no statistical covariance and no truth labels.
"""
import argparse,json,time,hashlib
from pathlib import Path
import numpy as np
from ndt_global.integrity import MapSupport,cluster
from geometry_ieskf.common import sha

def main(a):
 target=np.fromfile(a.data/'inputs/map_target.bin',dtype='<f4',offset=8).reshape(-1,3);ledger=[json.loads(l) for l in (a.data/'phase_aj_ndt_pose_update/ndt_transactions.jsonl').read_text().splitlines()];stream=json.loads((a.data/'history/phase_v/results/input_continuous_extension.json').read_text());center=np.array(ledger[0]['measurement']['final'])[:3,3]
 # Predeclared: remove XY-radius 20 m cylinder centered at first recorded candidate.
 # This is a controlled map fault; a historical match is not truth or a new seed.
 keep=np.linalg.norm(target[:,:2]-center[:2],axis=1)>20.;hole=target[keep];normal=MapSupport(target);damaged=MapSupport(hole);rows=[]
 for i in (0,71,142,213,283):
  f=stream['frames'][i];source=np.load(a.data/'history/phase_v'/f['source_file']);assert sha(a.data/'history/phase_v'/f['source_file'])==f['source_sha256'];T=ledger[i]['measurement']['final'];tick=time.perf_counter();base=normal.diagnose(source,T);m0=(time.perf_counter()-tick)*1000;tick=time.perf_counter();out=damaged.diagnose(source,T);m1=(time.perf_counter()-tick)*1000
  rows.append(dict(index=i,source_sha256=f['source_sha256'],control=base,injected_map_hole=out,control_ms=m0,injection_ms=m1,nearby_fraction_change=out.get('nearby_fraction',0)-base.get('nearby_fraction',0)))
 # Strip bases/eigenvectors: directional aggregate evidence is sufficient publicly.
 for r in rows:
  for k in ('control','injected_map_hole'):
   if r[k].get('geometry'):r[k]['geometry'].pop('basis',None)
 hist=json.loads((a.data/'phase_am_ndt_integrity/multistart_diagnostics.json').read_text());modes=[dict(index=i,classification=cluster(r['candidates'])['classification'],clusters=cluster(r['candidates'])['clusters'],converged=sum(c['converged'] and c.get('candidate_valid',True) for c in r['candidates']),fitness=[c.get('fitness') for c in r['candidates']],new_GPU_execution=False) for i,r in zip((0,71,142,213,283),hist)]
 a.output.write_text(json.dumps(dict(provenance='real historical frozen geometry with artificial Target deletion; old GPU results re-audited, not new fits',target_sha256=sha(a.data/'inputs/map_target.bin'),target_points=len(target),removed_target_points=int((~keep).sum()),remaining_target_points=len(hole),hole_radius_m=20.,geometry=rows,historical_multimode=modes,wrong_peak_labels='UNAVAILABLE',NDT_recomputed=False,GNSS_mode_selection='NOT_QUALIFIED',GPU_performance='NOT_MEASURED_THIS_PHASE',qualification='LIMITED_DIAGNOSTIC'),indent=2)+'\n')
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--data',type=Path,required=True);p.add_argument('--output',type=Path,required=True);main(p.parse_args())
