"""Verify saved sampling/index identities; never calls a matcher."""
import argparse,hashlib,json,struct
from pathlib import Path
import numpy as np
from sampling import space_order,SPACE_SEEDS
from run_ablation import read

def main(a):
 root=Path(a.root);work=root/'p4';primary=work/'ablation_01';supp=work/'stability_01';old={x['frame']:x for x in json.loads((root/'p2/five_01/execution_receipt.json').read_text())['records']};indices=np.load(primary/'retained_indices.npz');new=0
 for f in [60,64,65,66,76]:
  points=read(old[f]['request']['source_path'])
  for seed in SPACE_SEEDS:
   o=space_order(points,seed+f);np.testing.assert_array_equal(o[:20000],indices[f'{f}_space_{seed}'])
   for n,m in [(3000,5000),(5000,10000),(10000,20000)]:assert set(o[:n])<=set(o[:m])
  for folder in [primary,supp]:
   for row in json.loads((folder/'execution_receipt.json').read_text())['records']:
    if row['frame']!=f:continue
    assert row['result']['source_n']==row['n'] and row['result']['fitness_available']==False and row['result']['fitness'] is None and not row['result']['legacy_R_generated']
    if row['method']=='full':continue
    ix=indices[f"{f}_{row['method']}_{row['seed']}"][:row['n']];payload=np.ascontiguousarray(points[ix],dtype='<f4');assert hashlib.sha256(struct.pack('<Q',row['n'])+payload.tobytes()).hexdigest()==row['source_sha256'];new+=1
 assert new==70
 total=0
 for folder in [primary,supp]:
  ids=json.loads((folder/'identity_after.json').read_text());assert ids['unchanged'];total+=ids['calls']
  for name,h in ids['files'].items():assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==h
 assert total==72
 print('PASS: 72 calls; 70 subset byte identities; 15 source-only spatial orders and strict nesting; actual point counts/OFF semantics; immutable originals')
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--root',required=True);main(p.parse_args())
