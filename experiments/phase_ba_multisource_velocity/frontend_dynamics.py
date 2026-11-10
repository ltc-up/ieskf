"""Fixed-parameter response tests, actual signal analysis; no GNSS objective."""
import argparse,json
from pathlib import Path
import numpy as np
from geometry_ieskf.velocity_frontend import PolynomialVelocity
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);a=p.parse_args();out={};arrays={};dt=.004
for label,legacy in [('legacy',True),('PSD',False)]:
 f=PolynomialVelocity(legacy)
 for i in range(10000):f.push(i*dt,20.)
 F,Q=f.matrices(dt,legacy=legacy);P=F@f.P@F.T+Q;K=P[:,0]/(P[0,0]+f.r);A=F-np.outer(K,F[0]);freq=np.geomspace(.01,100,5000);G=np.array([(np.linalg.solve(np.eye(3)-A*np.exp(-2j*np.pi*h*dt),K))[0] for h in freq]);gain=abs(G);phase=np.unwrap(np.angle(G));cross=np.flatnonzero(gain<=1/np.sqrt(2));cutoff=float(freq[cross[0]]) if len(cross) else None
 out[label]=dict(nominal_Hz=250.,minus3dB_Hz=cutoff,gain_max=float(max(gain)),fixed_frequency={str(h):dict(gain=float(np.interp(h,freq,gain)),phase_degrees=float(np.interp(h,freq,phase)*180/np.pi),phase_delay_ms=float(-np.interp(h,freq,phase)/(2*np.pi*h)*1000)) for h in [.1,.5,1,2,5,10,20]})
 t=np.arange(0,6,dt);step=np.where(t<2,20.,10.);ramp=20.-10*np.clip(t-2,0,1);vals={}
 for name,signal in [('step',step),('brake_ramp',ramp),('lateral_sine',np.sin(2*np.pi*2*t))]:
  f=PolynomialVelocity(legacy);y=[]
  for ti,zi in zip(t,signal):f.push(float(ti),float(zi));y.append(f.query(float(ti))[0][0])
  y=np.array(y);mask=t>=2;vals[name]=dict(error_RMS=float(np.sqrt(np.mean((y[mask]-signal[mask])**2))),max_abs_error=float(max(abs(y[mask]-signal[mask]))))
  if name=='step':vals[name].update(t50_ms=float((t[np.flatnonzero((t>=2)&(y<=15))[0]]-2)*1000),t90_ms=float((t[np.flatnonzero((t>=2)&(y<=11))[0]]-2)*1000),undershoot=float(10-min(y[mask])))
  arrays[label+'_'+name]=y
 out[label]['deterministic_probes']=vals;arrays[label+'_gain']=gain
arrays['frequencies_Hz']=freq;arrays['time_s']=t;np.savez_compressed(a.root/'frontend_probes.npz',**arrays);(a.root/'frontend_dynamics.json').write_text(json.dumps(out,indent=2)+'\n')
