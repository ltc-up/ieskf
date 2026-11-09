"""Publish aggregate diagnostics only; full trajectories/state histories stay private."""
from pathlib import Path
import json,numpy as np
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path(__file__).resolve().parent;z=json.loads((p/'feedback_summary.json').read_text());cf=json.loads((p/'counterfactual_results.json').read_text());(p/'figures').mkdir(exist_ok=True);sections=z['sections'];labels=[str(v['section']) for v in sections]
fig,ax=plt.subplots(1,2,figsize=(10,4));ax[0].bar(labels,[v['gain_and_update']['g_lat']['mean'] for v in sections]);ax[0].set_ylabel('full unit lateral gain');x=np.arange(3);ax[1].bar(x-.15,[v['gain_and_update']['innovation_lat']['mean'] for v in sections],.3,label='innovation');ax[1].bar(x+.15,[v['gain_and_update']['exact_lat']['mean'] for v in sections],.3,label='base update');ax[1].set_xticks(x,labels);ax[1].set_ylabel('mean body-lateral m');ax[1].legend();fig.tight_layout();fig.savefig(p/'figures/gain_and_correction.png');plt.close(fig)
fig,ax=plt.subplots(figsize=(9,4))
for j,(key,label) in enumerate([('new_difference','motion minus NDT displacement'),('update','correction'),('basis_change','axis transport')]):ax.bar(np.arange(3)+(j-1)*.24,[v['balance'][key]['sum'] for v in sections],.24,label=label)
ax.set_xticks(np.arange(3),labels);ax.set_ylabel('sum of projected differences m (not true error)');ax.legend(fontsize=8);fig.tight_layout();fig.savefig(p/'figures/prediction_correction_balance.png');plt.close(fig)
fig,ax=plt.subplots(figsize=(8,4));rows=[r for r in cf['short_window_branches'] if r['case']=='skip_ndt_once'];ax.bar([str(r['section']) for r in rows],[r['next_prior_over_immediate_ratio'] for r in rows]);ax.axhline(1,color='k',ls='--');ax.set_ylabel('next-prior / immediate effect (fixed NDT)');ax.set_xlabel('predeclared representative section');fig.tight_layout();fig.savefig(p/'figures/feedback_persistence.png');plt.close(fig)
