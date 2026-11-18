"""Standalone B stream consumer for the authorized common causal replay runner.

Input NDJSON rows contain paths to ephemeral current dense/source arrays and a
causal independent U_T_base pose. Consume synchronously before scratch overwrite.
This consumer cannot recover missing IMU history and does not label a frozen
window as continuous replay. Reference trajectories are not accepted as input.
"""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
from bridge import ShadowGroup


def load(path, kind):
    if kind=='dense_xyzi':
        p=np.fromfile(path,dtype='<f4')
        if p.size%4:raise ValueError('invalid dense XYZI')
        return p.reshape(-1,4)[:,:3]
    if kind=='source_bin':
        with open(path,'rb') as f:
            n=np.fromfile(f,dtype='<u8',count=1)
            p=np.fromfile(f,dtype='<f4')
        if len(n)!=1 or p.size!=int(n[0])*3:raise ValueError('invalid source XYZ count')
        return p.reshape(-1,3)
    raise ValueError('unknown array format')


def main():
    a=argparse.ArgumentParser();a.add_argument('--service',required=True);a.add_argument('--scratch',required=True);a.add_argument('--variant',choices=['B-P','B0','B-M','B-K','B-K-MR'],required=True);a.add_argument('--output',required=True);args=a.parse_args()
    group=ShadowGroup(args.service,args.scratch,modes=(args.variant,))
    try:
        with open(args.output,'x') as out:
            for line in sys.stdin:
                r=json.loads(line)
                if r.get('kind')=='end':break
                if not r.get('causal_U') or not r.get('full_prefix_identity'):
                    raise ValueError('missing independent U or complete-prefix contract')
                if any(k in r for k in ('reference_pose','GNSS','global_final')):
                    raise ValueError('reference/global posterior not permitted in B input')
                result=group.on_frame(frame_id=r['frame_id'],ref_ns=r['ref_ns'],available_ns=r['available_ns'],dense=load(r['dense_path'],'dense_xyzi'),source=load(r['source_path'],'source_bin'),U_T_base=np.array(r['U_T_base']),deskew_history=r['deskew_history'])
                result['full_prefix_identity']=r['full_prefix_identity']
                out.write(json.dumps(result,allow_nan=False)+'\n');out.flush()
    finally:group.close()

if __name__=='__main__':main()
