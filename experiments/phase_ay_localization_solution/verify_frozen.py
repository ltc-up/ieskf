"""Check existing frozen payload hashes and all tracked original algorithm bytes."""
import hashlib,json,subprocess
from pathlib import Path
p=Path(__file__).resolve().parents[2]
def sha(f):
 h=hashlib.sha256()
 with Path(f).open('rb') as s:
  for b in iter(lambda:s.read(8*1024*1024),b''):h.update(b)
 return h.hexdigest()
at=json.loads((p/'experiments/phase_at_gnss_footprint_corners/manifest.json').read_text());b=json.loads((p/'data/phase_at_gnss_footprint_corners/bindings.json').read_text());checks={}
for k,v in at['input_sha256'].items():

 if not Path(b[k]).exists():checks['AT_'+k]={'status':'ORIGINAL_PATH_UNAVAILABLE','expected_sha256':v};continue
 actual=sha(b[k]);assert actual==v,k;checks['AT_'+k]=actual
for label,f,expected in [('AR_transactions','data/phase_ar_loop44_map0105/fullpose100_lap/transactions.json',at['AR_transaction_sha256']),('AS_F_transactions','data/phase_as_accuracy_quality/F_multilap/transactions.json',at['AS_F_transaction_sha256']),('AS_N_transactions','data/phase_as_accuracy_quality/N_lap/transactions.json',at['AS_N_transaction_sha256']),('frozen_config','data/history/phase_v/config/frozen.json','c595a50d62c841c8c6379a42c37bf6afc79592985daef8db77fbdff5c43da29f')]:
 # Native transaction location is recorded in the historical binding if linked differently.
 if not (p/f).exists() and label=='AS_N_transactions':checks[label]={'status':'ORIGINAL_PATH_UNAVAILABLE','expected_sha256':expected};continue
 actual=sha(p/f);assert actual==expected,label;checks[label]=actual
base='c647ac3224adee775d30dc6f9553c9b83612c060';names=subprocess.check_output(['git','ls-tree','-r','--name-only',base,'src','experiments/phase_ar_loop44_map0105','experiments/phase_as_accuracy_quality','experiments/phase_at_gnss_footprint_corners','experiments/phase_au_prediction_feedback'],cwd=p,text=True).splitlines()
for name in names:assert (p/name).read_bytes()==subprocess.check_output(['git','show',base+':'+name],cwd=p),name
print(json.dumps(dict(frozen_checks=checks,original_tracked_files_unchanged=len(names)),indent=2))
