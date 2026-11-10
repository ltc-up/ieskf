"""Aggregate spectra/gaps without publishing per-frame information or trajectories."""
import argparse,json
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--geometry',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();z=np.load(a.geometry);out={}
for name,key in [('NDT','ndt_info'),('GICP','gicp_info')]:
 eig=np.linalg.eigvalsh(z[key]);normalized=eig/eig.sum(axis=1)[:,None];gap=eig[:,1]/eig[:,0];out[name]=dict(trace_normalized_spectrum_median=np.median(normalized,axis=0).tolist(),spectrum_p05=np.percentile(normalized,5,axis=0).tolist(),spectrum_p95=np.percentile(normalized,95,axis=0).tolist(),lowest_pair_eigenvalue_ratio_median=float(np.median(gap)),single_eigenvector_uniqueness_not_assumed=True)
a.output.write_text(json.dumps(dict(aggregate_spectra=out,coordinate='shared frozen-seed axes; translation m, 10*rotation rad',not_noise_covariance=True),indent=2)+'\n')
