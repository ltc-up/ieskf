"""Explicit SYNTHETIC recovery fixture; never labeled a real GNSS trial."""
import json,collections
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from ndt_global.shadow import Contract,PoseHistory
from ndt_global.global_manager import GlobalManager,PositionObservation

def run():
    c=Contract('synthetic-map','synthetic-ext','synthetic-epoch');h=PoseHistory(c);g=GlobalManager(h)
    g.C[:3,:3]=Rotation.from_euler('z',.4).as_matrix();states=[g.status(0)['state']];steps=[];pruned=None
    def obs(t,p,bounds=False):return PositionObservation(t,'synthetic_gnss',str(t),p,c,fix_valid=True,datum_qualified=True,reference_qualified=True,clock_qualified=True,error_bounds_m=[.1]*3,bounds_qualified=bounds,source_independent_for_selection=bounds)
    local=[]
    for k in range(51):
        t=k*100_000_000;O=np.eye(4);O[0,3]=k*.2;h.add(t,O,c);local.append(O.copy())
        if k==2:
            A=O.copy();B=A.copy();B[0,3]+=5;g.ndt([A,B],'AMBIGUOUS',t,'synthetic_scan',t,c)
        if k in (0,1,3,30,31):
            before=[x.copy() for x in h.poses];e=g.position(obs(t,O[:3,3],k==3),t)
            for A,B in zip(before,h.poses):np.testing.assert_array_equal(A,B)
            steps.append(e)
            if k==3:pruned=e['remaining_hypotheses']
        states.append(g.status(t)['state'])
    assert pruned==1
    assert {'INITIALIZING','RECOVERING','PARTIAL','AMBIGUOUS','COASTING','UNAVAILABLE'}.issubset(states)
    assert abs(Rotation.from_matrix(g.C[:3,:3]).as_rotvec()[2]-.4)<1e-12
    # GLOBAL_TRACKING requires a separately qualified orientation fixture, not GNSS yaw.
    g2=GlobalManager(h,orientation_qualified=True)
    for k in (49,50):g2.position(obs(k*100_000_000,local[k][:3,3]),k*100_000_000)
    assert g2.status(5_000_000_000)['state']=='GLOBAL_TRACKING'
    return dict(evidence='SYNTHETIC_ONLY',state_counts=dict(collections.Counter(states)),separately_qualified_orientation_state='GLOBAL_TRACKING',
      position_anchors=len(steps),remaining_hypotheses_after_independent_bound=pruned,unobserved_yaw_error_rad=.4,
      local_history_unchanged=True,global_covariance='UNKNOWN_CORRELATION',statistical_fusion=False,
      real_sensor_claim=False,all_assertions_passed=True)
if __name__=='__main__':
    r=run();Path(__file__).with_name('synthetic_recovery.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
