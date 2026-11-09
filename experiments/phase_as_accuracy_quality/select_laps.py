"""Freeze production-only gate crossings before new multi-lap estimation."""
import argparse,csv,json
from pathlib import Path
import numpy as np
from ndt_global.track_evaluation import Polyline
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--center',type=Path,required=True);p.add_argument('--gnss',type=Path,required=True);p.add_argument('--private',type=Path,required=True);p.add_argument('--summary',type=Path,required=True);a=p.parse_args();a.private.mkdir(parents=True,exist_ok=True)
 data=json.loads(a.input.read_text());e=[e for e in data['events'] if e['kind']=='pose'];t=np.array([e['header_ns'] for e in e],dtype=np.int64);xy=np.array([e['position'][:2] for e in e]);c=np.genfromtxt(a.center,delimiter=',',names=True);line=Polyline(np.c_[c['Xref'],c['Yref']]);q=line.project(xy);anchor=1763205693.3654699;anchor_s=np.interp(anchor,(t-t[0])*1e-9+t[0]*1e-9,q['s']);s=q['s'];step=(np.diff(s)+line.length/2)%line.length-line.length/2;direction=1 if np.median(step[np.abs(step)>.01])>0 else -1;u=np.r_[s[0],s[0]+np.cumsum(step)];phase=direction*(u-anchor_s)/line.length;cross=np.where(np.floor(phase[1:])>np.floor(phase[:-1]))[0];g=[]
 for i in cross:
  if max(q['distance'][i:i+2])>20:continue
  level=np.floor(phase[i])+1;alpha=(level-phase[i])/(phase[i+1]-phase[i]);g.append(int(t[i]+round(alpha*(int(t[i+1])-int(t[i])))))
 truth=list(csv.DictReader(a.gnss.open()));gt=np.array([float(x['timestamp']) for x in truth]);gv=np.array([int(x['truth_valid']) for x in truth],bool);laps=[]
 for i,(lo,hi) in enumerate(zip(g,g[1:])):
  mask=(gt>=lo/1e9)&(gt<=hi/1e9);duration=(hi-lo)/1e9;valid=45<=duration<=130;laps.append(dict(id=i,start_ns=lo,end_ns=hi,duration_s=duration,usable=valid,reason='production fixed-gate complete crossing' if valid else 'outside frozen45..130s duration range',GNSS_rows=int(mask.sum()),GNSS_valid_fraction=float(gv[mask].mean()) if mask.any() else 0.))
 anchor_lap=min((v for v in laps if v['usable']),key=lambda v:abs(v['start_ns']/1e9-1763205693.3654699));chosen=[v for v in laps if anchor_lap['id']<=v['id']<=anchor_lap['id']+2];assert len(chosen)==3 and all(v['usable'] for v in chosen)
 for v in laps:v['selected']=v in chosen;v['selection_reason']='prior AR lap and next two complete production-gate laps' if v in chosen else 'outside preselected three-lap reference block; retained, not estimator-selected exclusion'
 out=dict(selection='fixed prior AR gate/production only; no new localization results',direction=direction,track_length_m=line.length,all_laps=laps,selected_ids=[v['id'] for v in chosen],selected_start_ns=chosen[0]['start_ns'],selected_end_ns=chosen[-1]['end_ns']);a.summary.write_text(json.dumps(out,indent=2)+'\n');(a.private/'multilap_window.txt').write_text('LAP_START='+str(chosen[0]['start_ns']/1e9)+'\nLAP_END='+str(chosen[-1]['end_ns']/1e9)+'\n');print(json.dumps(out,indent=2))
