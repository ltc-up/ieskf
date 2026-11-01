"""Plot public synthetic/deidentified aggregate AF results, no raw inputs."""
import argparse,json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--results',required=True);ap.add_argument('--output',required=True);a=ap.parse_args();v=json.load(open(a.results));s=v['synthetic'];f=v['frozen_summary'];fig,ax=plt.subplots(2,2,figsize=(11,8),constrained_layout=True)
 for q in s['observability']:
  ax[0,0].plot(range(1,7),q['singular_values'],'.-',label=q['kind'])
 ax[0,0].set(title='Synthetic surface Jacobian singular values',xlabel='Mode (1m / 1rad chart)',ylabel='Singular value');ax[0,0].legend(fontsize=7)
 d=s['density'];ax[0,1].semilogx([q['unique_samples'] for q in d],[q['Z_sigma'] for q in d],'.-',label='Independent Source + shared plane');ax[0,1].axhline(np.sqrt(d[0]['shared_plane_variance_floor']),color='k',linestyle='--',label='Shared-error floor');ax[0,1].set(title='Known generative surface law',xlabel='Distinct independent Source samples',ylabel='Conditional sigma Z (m)');ax[0,1].legend(fontsize=7)
 d=s['patch_partition'];xx=[q['patches'] for q in d];ax[1,0].plot(xx,[q['correct_shared_parent_Z_sigma'] for q in d],'.-',label='Same physical parent');ax[1,0].plot(xx,[q['naive_independent_patch_Z_sigma'] for q in d],'x--',label='Wrong: independent patch errors');ax[1,0].set(title='Same observations: only latent identity changes',xlabel='Patch aliases',ylabel='Conditional sigma Z (m)');ax[1,0].legend(fontsize=7)
 d=[q for q in f['candidate_scores'] if q['association']=='prediction_frozen'];ax[1,1].bar([q['candidate'] for q in d],[q['residual_abs_quantiles_m'][1] for q in d]);ax[1,1].set(title='Shared-data surface check, no accuracy ranking',ylabel='Common-factor P90 abs residual (m)');fig.savefig(Path(a.output),dpi=160)
if __name__=='__main__':main()
