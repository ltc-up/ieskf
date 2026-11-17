"""Pre-registered finite matrix. No reference reader and no production feedback."""
import json,subprocess,os,sys,time,hashlib
from pathlib import Path
root=Path(sys.argv[1]);exe=root/'build/joint/kg_s2';cfg=root/'inputs/series.json';ledger=[]
folder=sys.argv[2]if len(sys.argv)>2 else'results'
(root/folder).mkdir(exist_ok=False)if folder!='results' else None
for E in range(5):
 for label in ['fault','normal8','right3','ordinary','large_proxy']:
  dest=root/folder/f'{label}_E{E}';summary=root/folder/f'{label}_E{E}_summary.json'
  if summary.exists():
   ledger.append(dict(series=label,E=E,status='REUSED_ALREADY_COMPLETE',summary=str(summary)));continue
  start=time.time();dest.mkdir(exist_ok=False)
  with open(root/'logs'/f'{label}_E{E}.log','w')as log:
   try:
    z=subprocess.run([str(exe),str(cfg),label,str(E),str(dest),str(summary)],env={**os.environ,'OMP_NUM_THREADS':'2','OPENBLAS_NUM_THREADS':'1','KG_EXTENDED_AUDIT':'1'},stdout=log,stderr=log,timeout=610)
    status='COMPLETE'if z.returncode==0 else'FAILED'
   except subprocess.TimeoutExpired:status='RESOURCE_TIMEOUT'
  ledger.append(dict(series=label,E=E,status=status,wall_s=time.time()-start,binary_sha=hashlib.sha256(exe.read_bytes()).hexdigest()))
  (root/folder/'execution_ledger.json').write_text(json.dumps(ledger,indent=2))
  print(label,E,status,round(time.time()-start,2),flush=True)
  if status!='COMPLETE':raise RuntimeError('Preserved failed prefix; stop dependent matrix')
(root/folder/'execution_ledger.json').write_text(json.dumps(ledger,indent=2))
