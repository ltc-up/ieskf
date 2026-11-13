"""Plot aggregate engineering comparisons only; no raw trajectory is exported."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path(__file__).resolve().parent
runs=json.loads((p/'P0_CLOSED_LOOP_RESULTS.json').read_text())
fig,ax=plt.subplots(1,3,figsize=(13,3.6))
labels=['R3','R9','Normal5','Normal8'];x=np.arange(4)
for offset,name,label in [(-.18,'AY_lap','AY rerun'),(.18,'MT001_lap','Selected MT')]:
 v=runs[name]['sensitivity']['0.075']['corners']
 ax[0].bar(x+offset,[v[k]['lateral']['p95'] for k in ['3','9','5','8']],.36,label=label)
ax[0].set_xticks(x,labels);ax[0].set_ylabel('Lateral P95 (m)');ax[0].set_title('Native75ms engineering reference');ax[0].legend()
for name,label in [('AY_lap','AY rerun'),('MT001_lap','Selected MT')]:
 v=runs[name]['sensitivity'];ax[1].plot([0,75,150],[v[k]['xy']['rmse'] for k in ['0.0','0.075','0.15']],'o-',label=label)
ax[1].set_xlabel('Reference delay hypothesis (ms)');ax[1].set_ylabel('XY RMSE (m)');ax[1].set_title('Ranking reverses at150ms');ax[1].legend()
v=json.loads((p/'P0_SERIAL_PERFORMANCE.json').read_text())['cycle_ms'];ax[2].bar(['P50','P95','P99','max'],[v[k] for k in ['p50','p95','p99','max']],color='#597fa9');ax[2].axhline(50,color='#a33',ls='--',label='50ms deadline');ax[2].set_ylabel('Full processing cycle (ms)');ax[2].set_title('1468 frames, all overruns retained');ax[2].legend()
fig.suptitle('Conditional P0 gains; no safe rejection policy or production qualification',fontsize=11);fig.tight_layout();fig.savefig(p/'P0_OVERVIEW.png',dpi=170);plt.close(fig)
