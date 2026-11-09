"""Hash actual authorized analysis inputs, never publish their payload/path."""
import argparse,json,hashlib,subprocess
from pathlib import Path
p=argparse.ArgumentParser()
for k in ['at-bindings','ar-run','ar-bindings','as-f','as-n','config','output']:p.add_argument('--'+k,type=Path,required=True)
a=p.parse_args();root=Path(__file__).resolve().parents[2];at=json.loads((root/'experiments/phase_at_gnss_footprint_corners/manifest.json').read_text());binding=json.loads(a.at_bindings.read_text());checks={}
def sha(path):
 h=hashlib.sha256()
 with Path(path).open('rb') as f:
  for block in iter(lambda:f.read(8*1024*1024),b''):h.update(block)
 return h.hexdigest()
def check(name,path,expected):
 value=sha(path);assert value==expected,'FROZEN_HASH_MISMATCH:'+name;checks[name]=dict(sha256=value,matched=True)
for k,expected in at['input_sha256'].items():check('AT_'+k,binding[k],expected)
check('AR_transactions',a.ar_run/'transactions.json',at['AR_transaction_sha256']);check('AS_F_transactions',a.as_f,at['AS_F_transaction_sha256']);check('AS_N_transactions',a.as_n,at['AS_N_transaction_sha256']);check('AT_causal',Path(binding['private'])/'causal.npz',at['private_sha256']['causal.npz']);check('frozen_config',a.config,json.loads((Path(__file__).parent/'manifest.json').read_text())['config_sha256'])
unchanged=['src/geometry_ieskf','experiments/phase_ar_loop44_map0105','experiments/phase_as_accuracy_quality','experiments/phase_at_gnss_footprint_corners'];assert not subprocess.check_output(['git','diff','ac8e87f5755805c23c0dae305794000939a82299','--',*unchanged],cwd=root).strip()
a.output.write_text(json.dumps(dict(input_checks=checks,AR_states_used_sha256=sha(a.ar_run/'states.json'),AR_bindings_used_sha256=sha(a.ar_bindings),original_algorithm_and_reports_unchanged=True,bag_map='historical identities inherited; not read or rehashed by AU fixed-observation analysis'),indent=2)+'\n')
