"""Saved-arrival chronology only: virtual output grid, never estimates new states."""
import json,sys
from pathlib import Path
import numpy as np
from r_scale_counterfactual import stats
ORIGIN=1763205419863639180

def main(root,out):
 root=Path(root);out=Path(out);states=[json.loads(l) for l in (root/'eskf15_recorded_init/states.jsonl').open()];sims=np.array([r['sim_ns'] for r in states],dtype=np.int64);stamps=np.array([r['stamp_ns'] for r in states],dtype=np.int64);assert np.all(np.diff(sims)>=0)
 imu=[]
 for l in (root/'eskf15_recorded_init/inputs.jsonl').open():
  r=json.loads(l)
  if r['kind']=='imu':imu.append((r['actual_available_sim_ns'],r['header']))
 imu=np.array(imu,dtype=np.int64);assert np.all(np.diff(imu[:,0])>=0)
 start=ORIGIN+135_000_000_000;end=ORIGIN+345_806_063_080;grid=np.arange(start,end+1,10_000_000,dtype=np.int64);idx=np.searchsorted(sims,grid,side='right')-1;iidx=np.searchsorted(imu[:,0],grid,side='right')-1;valid=(idx>=0)&(iidx>=0);ids=idx[valid];ii=iidx[valid];g=grid[valid];ages=(g-stamps[ids])/1e6;behind=(imu[ii,1]-stamps[ids])/1e6;assert np.all(sims[ids]<=g)
 bins=np.floor_divide(sims-start,10_000_000);inside=(sims>=start)&(sims<=end);u,c=np.unique(bins[inside],return_counts=True)
 z=dict(scope='saved-record causal lookup on proposed 10ms grid, NOT timer execution or core-current-state reconstruction',saved_state_records=len(states),saved_causes={k:sum(r['cause']==k for r in states) for k in set(r['cause'] for r in states)},saved_record_interarrival_ms=stats(np.diff(sims)/1e6),saved_state_header_intervals_ms=stats(np.diff(stamps)/1e6),proposed_grid_slots=len(grid),slots_with_prior_saved_state=int(valid.sum()),slots_without_saved_state=int((~valid).sum()),saved_state_age_on_grid_ms=stats(ages),latest_delivered_imu_minus_saved_state_stamp_ms=stats(behind),reused_saved_snapshot_adjacent_slots=int((np.diff(ids)==0).sum()),bins_with_exactly_one_saved_record=int((c==1).sum()),bins_with_multiple_saved_records=int((c>1).sum()),bins_with_no_saved_record=int(len(grid)-len(u)),max_actual_sim_record_gap_ms=float(np.diff(sims).max()/1e6),causal_grid_lookup=True,new_predictions=0,new_observation_updates=0,unknown=['actual 100Hz publisher scheduling/jitter/deadlines; no timer publisher existed in C1','unlogged IMU-only intermediate core states and P','online revision atomicity under concurrent publishing'])
 (out/'output_schedule_audit.json').write_text(json.dumps(z,indent=2));print(json.dumps(z,indent=2))
if __name__=='__main__':main(*sys.argv[1:])
