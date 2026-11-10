"""Estimate a conditional noise proxy only on raw packets before first lap."""
import argparse,json
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--bindings',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--audit',type=Path,required=True);a=p.parse_args();assert not a.output.exists();d=json.loads(a.input.read_text());start=min(z['native_reference_ns'] for z in json.loads(a.bindings.read_text()));raw=[r for r in d['raw_topics']['/sensor/kistler/measurement'] if r['header_ns']<start];dd=np.diff(np.array([r['raw'] for r in raw]),n=2,axis=0);mid=np.median(dd,axis=0);mad=np.median(abs(dd-mid),axis=0)/.6744897501960817;z=np.clip((dd-mid)/mad,-3,3);corr=np.corrcoef(z.T);D=np.diag(mad/np.sqrt(6));R=D@corr@D;np.linalg.cholesky(R)
for e in d['events']:e['variance']=R.tolist()
a.output.write_text(json.dumps(dict(events=d['events']),separators=(',',':')));a.audit.write_text(json.dumps(dict(pre_run_raw_packets=len(raw),noise_proxy=R.tolist(),method='MAD(second difference)/sqrt6; clipped standardized correlation, pre-first-lap only',white_noise_qualification='CONDITIONAL: quantization/temporal dependence and true high-frequency dynamics not independently identified',common_error_prior_variance=.25,bias_process_variance=0,GNSS_used=False),indent=2)+'\n')
