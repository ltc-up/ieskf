"""KITTI0002 input contract. Navigation reference is segregated from online arrays.
No wheel surrogate, point firing-time fabrication, or production mutation.
"""
from pathlib import Path
import hashlib, json, struct
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from scipy.spatial import cKDTree

FIELDS = ('lat lon alt roll pitch yaw vn ve vf vl vu ax ay az af al au wx wy wz wf wl wu posacc velacc navstat numsats posmode velmode orimode').split()
MAP_IDS = tuple(range(20))
TEST_IDS = tuple(range(20,77))

def timestamps(path):
    rows=Path(path).read_text().splitlines()
    values=np.array([np.datetime64(s.strip().replace(' ','T'),'ns').astype(np.int64) for s in rows],dtype=np.int64)
    if len(values)<2 or np.any(np.diff(values)<=0): raise ValueError('NON_MONOTONIC_TIME')
    return values

def load_oxts(path):
    p=Path(path);files=sorted((p/'data').glob('*.txt'));t=timestamps(p/'timestamps.txt')
    if len(files)!=len(t): raise ValueError('OXTS_COUNT_TIME')
    v=np.array([np.loadtxt(f) for f in files])
    if v.shape!=(len(t),30) or not np.isfinite(v).all(): raise ValueError('OXTS_FIELDS')
    return t,v

def calibration(path):
    rows={}
    for line in Path(path).read_text().splitlines():
        k,v=line.split(':',1)
        try: rows[k]=np.array([float(x) for x in v.split()])
        except ValueError: pass
    T=np.eye(4);T[:3,:3]=rows['R'].reshape(3,3);T[:3,3]=rows['T']
    if not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-7) or np.linalg.det(T[:3,:3])<.999999:raise ValueError('CALIBRATION_SO3')
    return T

def reference_poses(packets):
    """KITTI devkit Mercator convention; output map(ENU-like)-from-IMU.
    Offline mapping/evaluation ONLY. Never called by run-time propagation.
    """
    v=np.asarray(packets); scale=np.cos(np.deg2rad(v[0,0]));er=6378137.
    p=np.column_stack((scale*np.deg2rad(v[:,1])*er,scale*er*np.log(np.tan(np.deg2rad(90+v[:,0])/2)),v[:,2]))
    p-=p[0]
    T=np.repeat(np.eye(4)[None],len(v),axis=0);T[:,:3,3]=p
    # Extrinsic xyz equals Rz(yaw) Ry(pitch) Rx(roll); differs from NDT intrinsicXYZ.
    T[:,:3,:3]=Rotation.from_euler('xyz',v[:,3:6]).as_matrix()
    return T

def reference_at(raw_t, packets, query_t):
    """Offline map/evaluation interpolation at actual scan epoch, never index-pair.
    Integer subtraction before float conversion preserves timestamp precision.
    """
    raw_t=np.asarray(raw_t,dtype=np.int64);query_t=np.asarray(query_t,dtype=np.int64)
    if query_t.min()<raw_t[0] or query_t.max()>raw_t[-1]:raise ValueError('REFERENCE_NO_BRACKET')
    A=reference_poses(packets);tr=(raw_t-raw_t[0])*1e-9;tq=(query_t-raw_t[0])*1e-9
    T=np.repeat(np.eye(4)[None],len(query_t),axis=0)
    T[:,:3,:3]=Slerp(tr,Rotation.from_matrix(A[:,:3,:3]))(tq).as_matrix()
    for k in range(3):T[:,k,3]=np.interp(tq,tr,A[:,k,3])
    T[:,:3,3]-=T[0,:3,3].copy()
    return T

def transform(points,T):return np.asarray(points)@T[:3,:3].T+T[:3,3]

def write_cloud(path,points):
    points=np.asarray(points,dtype='<f4')
    with Path(path).open('xb') as f:f.write(struct.pack('<Q',len(points)));points.tofile(f)

def read_velodyne(path):
    p=Path(path)
    if p.stat().st_size%16:raise ValueError('POINT_STRIDE')
    v=np.fromfile(p,dtype='<f4').reshape(-1,4)
    if not len(v) or not np.isfinite(v).all():raise ValueError('POINT_NONFINITE_OR_EMPTY')
    return v

def stat(a):
    a=np.asarray(a);return dict(min=float(a.min()),p50=float(np.median(a)),p95=float(np.percentile(a,95)),max=float(a.max()))

def prepare(raw,out):
    raw=Path(raw);out=Path(out);out.mkdir(exist_ok=False)
    date=raw/'2011_09_26';sync=date/'2011_09_26_drive_0002_sync';extract=date/'2011_09_26_drive_0002_extract'
    st,sv=load_oxts(sync/'oxts');it,iv=load_oxts(extract/'oxts');lt=timestamps(sync/'velodyne_points/timestamps.txt')
    starts=timestamps(sync/'velodyne_points/timestamps_start.txt');ends=timestamps(sync/'velodyne_points/timestamps_end.txt')
    if len(lt)!=77 or len(it)!=846 or len(st)!=77:raise ValueError('APPROVED_SCENE_COUNT')
    if not (np.all(starts<=lt) and np.all(lt<=ends)):raise ValueError('SCAN_TIME_ORDER')
    if it[0]>lt[0] or it[-1]<lt[-1]:raise ValueError('IMU_COVERAGE')
    T_VI=calibration(date/'calib_imu_to_velo.txt');T_IV=np.linalg.inv(T_VI);ref=reference_at(it,iv,lt)
    # Online initialization uses mapping-frame0 only, not a test reference.
    online=dict(imu_t=it,omega=iv[:,17:20],force=iv[:,11:14],scan_t=lt,scan_start=starts,scan_end=ends,
                initial_p=ref[0,:3,3],initial_R=ref[0,:3,:3],initial_v=np.array([np.interp(float(lt[0]-it[0]),(it-it[0]).astype(float),iv[:,k]) for k in (7,6,10)]),
                T_IMU_velo=T_IV,initial_t=np.array(lt[0],dtype=np.int64))
    np.savez_compressed(out/'online.npz',**online)
    np.savez_compressed(out/'offline_reference.npz',t=lt,poses=ref,sync_t=st,nav_packets=sv,raw_t=it,raw_packets=iv)
    sources=out/'sources';sources.mkdir();mapping=[];pointcounts=[];sha=hashlib.sha256()
    for i in range(77):
        path=sync/'velodyne_points/data'/f'{i:010d}.bin';v=read_velodyne(path);sha.update(path.read_bytes());p=transform(v[:,:3],T_IV)
        write_cloud(sources/f'{i:03d}.bin',p);pointcounts.append(len(p))
        if i in MAP_IDS:mapping.append(transform(p,ref[i]))
    points=np.concatenate(mapping);_,idx=np.unique(np.floor(points/.2).astype(np.int64),axis=0,return_index=True);target=points[np.sort(idx)]
    write_cloud(out/'map.bin',target)
    tree=cKDTree(target);coverage=[]
    # Reference use here is OFFLINE map-overlap measurement, never a match seed/gate.
    for i in TEST_IDS:
        v=read_velodyne(sync/'velodyne_points/data'/f'{i:010d}.bin');q=transform(transform(v[:,:3],T_IV),ref[i]);q=q[np.linspace(0,len(q)-1,min(4096,len(q)),dtype=int)]
        dist,_=tree.query(q,workers=1);coverage.append(dict(frame=i,samples=len(q),fraction_within_1m=float(np.mean(dist<1)),fraction_within_2m=float(np.mean(dist<2)),median_nn_m=float(np.median(dist)),coverage_qualified=bool(np.mean(dist<1)>=.5)))
    Rraw=Rotation.from_euler('xyz',iv[:,3:6]).as_matrix();earth_acc=np.einsum('nij,nj->ni',Rraw,iv[:,11:14])+[0,0,-9.80665]
    navvel=np.column_stack((iv[:,7],iv[:,6],iv[:,10]));dt=np.diff(it)*1e-9;dv=np.diff(navvel,axis=0)/dt[:,None]
    flags=iv[:,27:30];gaps=np.where(dt>.025)[0]
    audit=dict(imu_records=len(it),lidar_records=len(lt),sync_oxts=len(st),imu_interval_s=stat(dt),lidar_interval_s=stat(np.diff(lt)*1e-9),scan_duration_s=stat((ends-starts)*1e-9),
       sync_oxts_minus_lidar_ms=stat((st-lt)*1e-6),nearest_raw_imu_offset_ms=stat(np.min(abs(it[:,None]-lt[None,:]),axis=0)*1e-6),
       imu_gap_indices=gaps.tolist(),invalid_navigation_mode_records=int(np.any(flags<0,axis=1).sum()),nav_mode_unique=np.unique(flags,axis=0).tolist(),
       body_force_mean=iv[:,11:14].mean(axis=0).tolist(),body_force_norm=stat(np.linalg.norm(iv[:,11:14],axis=1)),body_angular_rate_norm=stat(np.linalg.norm(iv[:,17:20],axis=1)),
       offline_gravity_consistency_world_acc_mean=earth_acc.mean(axis=0).tolist(),offline_velocity_derivative_discrepancy_rmse=np.sqrt(np.mean((dv-earth_acc[:-1])**2,axis=0)).tolist(),
       input_inertial_channels='exported OXTS body ax/ay/az and wx/wy/wz, not raw ADC; navigation/receiver correction dependence unknown',
       point_fields=['x','y','z','reflectance'],per_point_time_available=False,additional_deskew=False,exact_scan_motion_compensation_unknown=True,
       frame_contract='source points explicitly converted Velodyne->IMU; NDT map-from-IMU, ExternalPose zero lever; no claim rear axle',
       map_indices=list(MAP_IDS),test_indices=list(TEST_IDS),map_raw_points=len(points),map_voxel_m=.2,map_points=len(target),point_counts=pointcounts,
       source_stream_sha256=sha.hexdigest(),imu_calibration_sha256=hashlib.sha256((date/'calib_imu_to_velo.txt').read_bytes()).hexdigest(),
       map_target_sha256=hashlib.sha256((out/'map.bin').read_bytes()).hexdigest(),coverage=coverage,
       reference_alignment='raw OXTS pose interpolated at actual LiDAR epoch; sync rows NOT index-paired',initial_reference_frame=0,test_reference_online=False,wheel_enabled=False,gnss_online_updates=False,C_fusion=False,A_fusion=False,
       raw_time_range_ns=[int(it[0]),int(it[-1])],lidar_time_range_ns=[int(lt[0]),int(lt[-1])])
    (out/'input_audit.json').write_text(json.dumps(audit,indent=2)+'\n')
    (out/'config.json').write_text(json.dumps(dict(map_indices=MAP_IDS,test_indices=TEST_IDS,map_voxel_m=.2,ndt=dict(resolution=1.5,step_size=.2,max_iterations=35,transformation_epsilon=.01,relative_objective_epsilon=.001,neighbor='DIRECT7',line_search='GLL3',q=50),baseline='143ce47adfb60cbc8d1f069d4945b33641e1ca96'),indent=2)+'\n')
    return audit

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--raw',required=True);p.add_argument('--out',required=True);a=p.parse_args();v=prepare(a.raw,a.out)
    print(json.dumps({k:v[k] for k in ['imu_records','lidar_records','imu_interval_s','body_force_mean','map_points']},indent=2))
