"""Validate already-frozen inputs only. No ROS publication, matching or state update."""
import argparse,json,time,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from input_adapter import decode_cloud,fuse_three,adapt_imu


def stats(x):
    return dict(zip(('p50','p95','p99','max'),np.percentile(x,[50,95,99,100]).tolist())) if len(x) else None


def main(a):
    data=Path(a.data);selection=json.loads(Path(a.selection).read_text());out=Path(a.out)
    if out.exists():raise ValueError('REFUSE_OVERWRITE')
    bindings=json.loads((data/'phase_ar_loop44_map0105/bindingslap.json').read_text())
    if hashlib.sha256((data/'phase_ar_loop44_map0105/bindingslap.json').read_bytes()).hexdigest()!=selection['bindings_sha256']:
        raise ValueError('FROZEN_BINDINGS_IDENTITY_MISMATCH')
    motion=json.loads((data/'phase_ao_ap_validation/loop44.json').read_text())
    from_tf={}
    edges={}
    for e in motion['transforms']:
        T=np.eye(4);T[:3,:3]=Rotation.from_quat(e['q']).as_matrix();T[:3,3]=e['p']
        edges.setdefault(e['parent'],[]).append((e['child'],T));edges.setdefault(e['child'],[]).append((e['parent'],np.linalg.inv(T)))
    def tf(child):
        todo=[('base_link',np.eye(4))];seen=set()
        while todo:
            name,T=todo.pop()
            if name==child:return T
            if name in seen:continue
            seen.add(name);todo.extend((c,T@A) for c,A in edges.get(name,[]) if c not in seen)
        raise ValueError('TF_MISSING')
    Tbi=tf('imu_bosch');rows=[];failures=[];imus=[]
    for w in selection['windows']:
        for x in w['clouds']:
            i=x['index'];job=bindings[i];t0=time.perf_counter();clouds=[]
            try:
                for c in job['clouds']:
                    ident=next(y for y in x['inputs'] if y['message_id']==c['message_id'])
                    if c['raw_sha']!=ident['sha256']:raise ValueError('IDENTITY_MISMATCH')
                    sid={'lidar_front':0,'lidar_left':1,'lidar_right':2}[c['frame']]
                    T=tf(c['frame'])
                    if not np.allclose(T,c['T_body_lidar'],atol=1e-12):raise ValueError('STATIC_TF_MISMATCH')
                    raw=data/Path(c['raw_path']).relative_to('data')
                    payload=raw.read_bytes()
                    clouds.append(decode_cloud(payload,c['fields'],c['width']*c['height'],c['point_step'],c['header_ns'],c['arrival_ns'],sid,c['message_id'],T,c['bigendian']))
                read_ms=(time.perf_counter()-t0)*1e3;start=time.perf_counter();f,m=fuse_three(clouds);fusion_ms=(time.perf_counter()-start)*1e3
                if m['max_point_time_roundtrip_error_ns']>4:raise ValueError('POINT_TIME_ROUNDTRIP_OVER_4NS')
                rows.append(dict(window=w['name'],index=i,n=len(f),per_sensor=[len(c.xyz) for c in sorted(clouds,key=lambda c:c.sensor_id)],point_time_error_ns=m['max_point_time_roundtrip_error_ns'],header_minus_existing_ns=m['header_ns']-job['header'],available_minus_header_ms=(m['available_ns']-m['header_ns'])*1e-6,read_ms=read_ms,fusion_ms=fusion_ms))
            except Exception as e:failures.append(dict(window=w['name'],index=i,reason=str(e)))
        lo=bindings[w['warmup_indices'][0]]['header']-100_000_000;hi=bindings[w['score_indices'][1]]['header']
        ev=[e for e in motion['events'] if e['kind']=='imu' and lo<=e['header_ns']<=hi]
        previous=None
        for e in ev:
            try:
                z=adapt_imu(e['omega'],e['force'],Tbi,e['header_ns'],previous)
                imus.append(dict(window=w['name'],force_norm=float(np.linalg.norm(z['force_base'])),lever_norm=float(np.linalg.norm(z['lever_correction'])),alpha_norm=float(np.linalg.norm(z['angular_acceleration_base'])),gyro_norm=float(np.linalg.norm(z['omega_base'])),stamp_ns=e['header_ns']))
                previous=z
            except ValueError as err:
                if previous is None and str(err)=='PREVIOUS_GYRO_REQUIRED_FOR_LEVER_ARM':
                    previous=dict(stamp_ns=e['header_ns'],omega_base=Tbi[:3,:3]@e['omega'])
                else:failures.append(dict(window=w['name'],imu_stamp=e['header_ns'],reason=str(err)));previous=None
    result=dict(mode='NO_ROS_NO_MATCHING_NO_STATE_UPDATE',groups_planned=298,groups_valid=len(rows),failures=failures,windows={})
    for w in selection['windows']:
        q=[r for r in rows if r['window']==w['name']];v=[r for r in imus if r['window']==w['name']]
        result['windows'][w['name']]=dict(groups=len(q),point_count=stats([r['n'] for r in q]),read_ms=stats([r['read_ms'] for r in q]),fusion_ms=stats([r['fusion_ms'] for r in q]),available_minus_header_ms=stats([r['available_minus_header_ms'] for r in q]),point_time_error_max_ns=max([r['point_time_error_ns'] for r in q],default=None),header_minus_existing_ns=stats([r['header_minus_existing_ns'] for r in q]),imu_samples=len(v),lever_acceleration_mps2=stats([r['lever_norm'] for r in v]),alpha_radps2=stats([r['alpha_norm'] for r in v]),force_norm_mps2=stats([r['force_norm'] for r in v]))
    out.mkdir();(out/'summary.json').write_text(json.dumps(result,indent=2)+'\n');(out/'fusion_rows.json').write_text(json.dumps(rows)+'\n');(out/'imu_diagnostics.json').write_text(json.dumps(imus)+'\n');print(json.dumps(result,indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--data',required=True);p.add_argument('--selection',required=True);p.add_argument('--out',required=True);main(p.parse_args())
