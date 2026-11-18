"""Synthetic only end-to-end exercise. Never an accuracy/real-time certification."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
from bridge import ShadowGroup


def main():
    p=argparse.ArgumentParser();p.add_argument('--service',required=True);p.add_argument('--output',required=True);p.add_argument('--scratch',required=True);a=p.parse_args()
    rng=np.random.default_rng(42)
    # Nonrepeating 3D clusters permit min_points=6 Gaussian voxels after .8 m.
    centers=rng.uniform([-40,-25,2],[40,25,12],size=(40,3))
    world=np.vstack([c+rng.normal(0,1.5,size=(1000,3)) for c in centers])
    group=ShadowGroup(a.service,a.scratch,modes=('B-P','B0','B-M','B-K','B-K-MR'))
    rows=[]
    try:
        for i in range(6):
            T=np.eye(4);T[0,3]=i*.3
            points=world-T[:3,3]
            source=points[::8][:5000]
            rows.append(group.on_frame(frame_id=i,ref_ns=(i+1)*50_000_000,available_ns=(i+1)*50_000_000+1000,dense=points,source=source,U_T_base=T,deskew_history='SYNTHETIC; no map feedback'))
    finally:group.close()
    out=Path(a.output);out.write_text(json.dumps(dict(kind='synthetic_gpu_not_real_data',seed=42,frames=rows),indent=2,allow_nan=False))
    print(json.dumps(dict(output=str(out),statuses={m:[r['variants'][m]['status'] for r in rows] for m in group.chains},p50_full_step_ms={m:float(np.median([r['variants'][m]['full_step_ms'] for r in rows[1:]])) for m in group.chains})))

if __name__=='__main__':main()
