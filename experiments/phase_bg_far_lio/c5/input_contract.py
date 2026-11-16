"""Reuse C1 decoder, change only the explicitly frozen end boundary in memory."""
import json,sys
from pathlib import Path
REPO=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(REPO/'experiments/phase_bg_far_lio/c1')]
import continuous_inputs as ci

def configure(root):
 c=json.load(open(Path(root)/'config.json'));ci.START=c['start_ns'];ci.INIT=c['init_ns'];ci.END=c['end_ns'];return c
if __name__=='__main__':
 import argparse
 from types import SimpleNamespace
 p=argparse.ArgumentParser();p.add_argument('root');p.add_argument('mode',choices=['identity','realtime']);a=p.parse_args();c=configure(a.root);args=SimpleNamespace(root=a.root,bag=c['bag'],audit=c['audit'],extrinsics=c['extrinsics'])
 if a.mode=='identity':ci.audit_stream(args)
 else:
  import realtime_contract;realtime_contract.main(args)
