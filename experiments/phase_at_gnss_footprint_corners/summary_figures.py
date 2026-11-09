"""De-identified aggregate figures; no map or private full trajectories."""
from pathlib import Path
import json,numpy as np
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path(__file__).resolve().parent;z=json.loads((p/'statistics.json').read_text());(p/'figures').mkdir(exist_ok=True)
f,ax=plt.subplots(figsize=(8,4));names=['prior','ndt','posterior','production']
for j,n in enumerate(names):ax.bar(np.arange(3)+j*.18,[v['error'][n]['p95'] for v in z['corners']],.18,label=n)
ax.set_xticks(np.arange(3)+.27,['control1','right3','right9']);ax.set_ylabel('engineering GNSS lateral P95 m');ax.legend();f.tight_layout();f.savefig(p/'figures/corner_decomposition.png');plt.close(f)
f,ax=plt.subplots(figsize=(8,4))
for kind in ['A_curve_query_shift','B_raw_gps_delay']:
 for name in ['A','B']:ax.plot([0,75,150],[z['time_sensitivity'][str(v)][kind][name]['XY']['rmse'] for v in [0.,.075,.15]],'o-',label=kind+':'+name)
ax.set_xlabel('fixed GPS delay hypothesis ms');ax.set_ylabel('engineering GNSS XY RMSE m');ax.legend(fontsize=7);f.tight_layout();f.savefig(p/'figures/time_sensitivity.png');plt.close(f)
f,ax=plt.subplots();keys=list(z['polygon_results']);ax.bar(['15D','production','GNSS'],[z['polygon_results'][k]['boundary_separation_m']['min'] for k in keys]);ax.set_ylabel('conditional visual-box minimum clearance m');ax.set_title('Not measured tyre/body safety');f.tight_layout();f.savefig(p/'figures/conditional_clearance.png');plt.close(f)
