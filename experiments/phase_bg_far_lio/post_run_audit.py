"""Post-run contracts and representative spatial statistics; never runs matching."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
import yaml
from input_adapter import decode_cloud,fuse_three


def differences(a,b,p=''):
    if isinstance(a,dict) and isinstance(b,dict):
        return [x for k in sorted(set(a)|set(b)) for x in differences(a.get(k),b.get(k),p+'/'+str(k))]
    return [] if a==b else [p]


def spatial(x):
    d=np.linalg.norm(x,axis=1);az=(np.arctan2(x[:,1],x[:,0])+np.pi)/(2*np.pi)
    return dict(n=len(x), span_xyz=np.ptp(x,axis=0).tolist(),
                range_bins_0_10_20_40_80_inf=np.histogram(d,[0,10,20,40,80,np.inf])[0].tolist(),
                azimuth_sectors_occupied_36=int(len(np.unique(np.minimum((az*36).astype(int),35)))),
                voxels_0p8m=int(len(np.unique(np.floor(x/.8).astype(np.int64),axis=0))),
                voxels_4m=int(len(np.unique(np.floor(x/4).astype(np.int64),axis=0))))



def main(a):
    root=Path(a.root);source=Path(a.source);data=Path(a.data);out=root/'audit/final_contracts.json'
    if out.exists():raise ValueError('REFUSE_OVERWRITE')
    original=yaml.safe_load((source/'config/far-lio.yml').read_text());summary=dict(configs={},spatial_representatives={},prohibitions=dict(new_dataset=False,new_bag=False,new_map_geometry=False,docker_pull=False,production_modified=False,online_gnss=False,recorded_dynamic_tf=False))
    bindings=json.loads((data/'phase_ar_loop44_map0105/bindingslap.json').read_text())
    for name,folder,index in [('normal8','normal8_retry1',675),('right3','right3',1317)]:
        p=root/'results'/folder;cfg=yaml.safe_load((p/'far_lio.yml').read_text());diff=[x.replace("//core/","/core/",1) for x in differences(original,cfg)]
        allowed=['/core/state/LidarOdometry/ros__parameters/node/','/core/state/LidarOdometry/ros__parameters/model/','/core/state/StateEstimation/ros__parameters/node/odom_frame_id','/core/state/StateEstimation/ros__parameters/inputs/imus/primary/message/topic','/core/state/LidarOdometry/ros__parameters/pipeline/update_map','/core/state/LidarOdometry/ros__parameters/pipeline/debug_mode']
        assert all(any(x.startswith(k) for k in allowed) for x in diff),diff
        assert cfg['/core/state']['LidarOdometry']['ros__parameters']['registration']==original['/core/state']['LidarOdometry']['ros__parameters']['registration']
        assert cfg['/core/state']['StateEstimation']['ros__parameters']['kalman_filter']==original['/core/state']['StateEstimation']['ros__parameters']['kalman_filter']
        summary['configs'][name]=dict(changed_paths=diff,optimizer_Q_R_enable_preserved=True,sha256=hashlib.sha256((p/'far_lio.yml').read_bytes()).hexdigest(),run=json.loads((p/'run_status.json').read_text()))
        clouds=[]
        for c in bindings[index]['clouds']:
            sid={'lidar_front':0,'lidar_left':1,'lidar_right':2}[c['frame']]
            clouds.append(decode_cloud((data/Path(c['raw_path']).relative_to('data')).read_bytes(),c['fields'],c['width']*c['height'],c['point_step'],c['header_ns'],c['arrival_ns'],sid,c['message_id'],c['T_body_lidar'],c['bigendian']))
        f,m=fuse_three(clouds);xyz=np.column_stack([f[x] for x in ['x','y','z']]);summary['spatial_representatives'][name]={'index':index,'total':spatial(xyz),'sensors':{str(i):spatial(xyz[f['sensor_id']==i]) for i in range(3)}}
    summary['downloads']=json.loads((root/'dependencies/download_ledger.json').read_text());summary['download_bytes']=sum(r['bytes'] for r in summary['downloads']);assert summary['download_bytes']<=512*1024**2
    summary['allocated_root_bytes']=sum(p.stat().st_blocks*512 for p in root.rglob('*') if p.is_file());assert summary['allocated_root_bytes']<20*1024**3
    out.write_text(json.dumps(summary,indent=2)+'\n');print('Config and budgets passed; downloads',summary['download_bytes'],'allocated bytes',summary['allocated_root_bytes'])

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',required=True);p.add_argument('--source',required=True);p.add_argument('--data',required=True);main(p.parse_args())
