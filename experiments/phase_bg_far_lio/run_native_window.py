"""One preregistered ROS window, original arrival schedule; no reference inputs.

Only our two child processes are terminated. Original dynamic TF is never read or
published. Research products are private; no raw point payload is logged.
"""
import argparse,hashlib,json,os,signal,subprocess,threading,time
from pathlib import Path
import numpy as np
import yaml
from input_adapter import decode_cloud,fuse_three,adapt_imu


def json_scalar(x):
    if isinstance(x,bytes) and len(x)==1:return int.from_bytes(x,'little')
    if isinstance(x,np.generic):return x.item()
    raise TypeError(type(x).__name__)

def ns(t):return int(t.sec)*10**9+int(t.nanosec)
def stamp(t,n):t.sec=int(n)//10**9;t.nanosec=int(n)%10**9


def main(a):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from sensor_msgs.msg import Imu,PointCloud2,PointField
    from rosgraph_msgs.msg import Clock
    from nav_msgs.msg import Odometry
    from diagnostic_msgs.msg import DiagnosticArray
    from tsl_msgs.msg import TSLDefinition,TSLValues
    from tf2_ros import StaticTransformBroadcaster
    from geometry_msgs.msg import TransformStamped
    from scipy.spatial.transform import Rotation
    root=Path(a.root);data=Path(a.data);source=Path(a.source);out=root/'results'/(a.window+('_'+a.attempt if a.attempt else ''))
    if out.exists():raise RuntimeError('REFUSE_WINDOW_RERUN_OR_OVERWRITE')
    sel=json.loads((root/'audit/frozen_windows.json').read_text());w=next(x for x in sel['windows'] if x['name']==a.window)
    valid=json.loads((root/'results/input_validation/summary.json').read_text())
    if valid['failures'] or valid['groups_valid']!=298:raise RuntimeError('INPUT_CONTRACT_FAILED')
    build=json.loads((root/'audit/build_result_utf8.json').read_text())
    if build['returncode']!=0:raise RuntimeError('BUILD_FAILED')
    bindings=json.loads((data/'phase_ar_loop44_map0105/bindingslap.json').read_text())
    if hashlib.sha256((data/'phase_ar_loop44_map0105/bindingslap.json').read_bytes()).hexdigest()!=sel['bindings_sha256']:raise ValueError('INPUT_IDENTITY')
    motion=json.loads((data/'phase_ao_ap_validation/loop44.json').read_text());audit=json.loads((root/'audit/bag_one_second.json').read_text())
    Tbi=np.array(json.loads((root.parent/'phase_be_e0_e4/audit/be5_stage1/source_and_contract_evidence.json').read_text())['T_base_imu'])
    init=json.loads((root/'runtime/input_contract/initialization.json').read_text())[a.window]
    first=bindings[w['warmup_indices'][0]];last=bindings[w['score_indices'][1]]
    lo=min(c['scan_start_ns'] for c in first['clouds'])-50_000_000
    hi=max(c['arrival_ns'] for c in last['clouds'])+50_000_000
    ev=[]
    for e in motion['events']:
        if e['kind']=='imu' and lo<=e['header_ns']<=hi:
            ev.append((int(e['recorder_ns']),0,e))
    for x in w['clouds']:
        job=bindings[x['index']];ev.append((max(c['arrival_ns'] for c in job['clouds']),1,dict(index=x['index'],job=job)))
    ev.sort(key=lambda x:(x[0],x[1]));out.mkdir()
    cfg=yaml.safe_load((source/'config/far-lio.yml').read_text());section=cfg['/core/state']
    lp=section['LidarOdometry']['ros__parameters'];sp=section['StateEstimation']['ros__parameters']
    lp['pipeline']['update_map']=False;lp['pipeline']['debug_mode']=False
    lp['node'].update(odom_frame='map',child_frame='base_link',cloud_frame='base_link',input_pointcloud='/bg/input/fused',input_map=str(root/'runtime/input_contract/existing_target_xyz.pcd'),update_map_srv='',wait_tf=False)
    pos=init['position'];q=init['quaternion_xyzw'];lp['model'].update(dict(zip(['initial_pos_x','initial_pos_y','initial_pos_z'],pos)));lp['model'].update(dict(zip(['initial_rot_x','initial_rot_y','initial_rot_z','initial_rot_w'],q)))
    sp['node']['odom_frame_id']='map';sp['inputs']['imus']['primary']['message']['topic']='/bg/input/imu'
    # Do not add auxiliary measurements; retain the official internal tilt and Q/R.
    if set(sp['measurements'])!={'lidar_gicp'} or set(sp['inputs']['imus'])!={'primary'}:raise ValueError('AUXILIARY_INPUT_NOT_ALLOWED')
    config=out/'far_lio.yml';config.write_text(yaml.safe_dump(cfg,sort_keys=False))
    install=root/'runtime/utf8/install';binaries=[install/'tam_odometry/lib/tam_odometry/tam_cuda_gicp_ext',install/'tam_state_estimation_node/lib/tam_state_estimation_node/tam_state_estimation_node_ekf_3d_nh']
    manifest=dict(window=w,config_sha256=hashlib.sha256(config.read_bytes()).hexdigest(),binaries={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in binaries},domain=os.environ.get('ROS_DOMAIN_ID'),map_identity=json.loads((root/'runtime/input_contract/map_identity.json').read_text()),init=init,reference_inputs=False,recorded_dynamic_tf=False,upstream_deskew=False,internal_tilt_reference=sp['enable']['reference_angle_measurement'])
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    rclpy.init();node=rclpy.create_node('bg_frozen_inputs');executor=SingleThreadedExecutor();executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    best=QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT)
    offered=QoSProfile(depth=20,reliability=ReliabilityPolicy.RELIABLE)
    imu_pub=node.create_publisher(Imu,'/bg/input/imu',offered);cloud_pub=node.create_publisher(PointCloud2,'/bg/input/fused',offered);clock_pub=node.create_publisher(Clock,'/clock',10)
    now={'sim':ev[0][0]-1_000_000};lock=threading.Lock();records=[];definitions={};inputs=[]
    record_stream=(out/'records.jsonl').open('x')
    def store(kind,**kw):
        row=dict(kind=kind,received_wall_ns=time.monotonic_ns(),sim_ns=now['sim'],**kw)
        line=json.dumps(row,default=json_scalar)+'\n'
        with lock:
            records.append(row);record_stream.write(line);record_stream.flush()
    def odom_cb(m,kind):
        p=m.pose.pose.position;q=m.pose.pose.orientation;v=m.twist.twist.linear
        store(kind,stamp_ns=ns(m.header.stamp),frame=m.header.frame_id,child=m.child_frame_id,position=[p.x,p.y,p.z],quaternion=[q.x,q.y,q.z,q.w],velocity=[v.x,v.y,v.z],pose_cov=list(m.pose.covariance))
    subs=[node.create_subscription(Odometry,'/core/state/lidar_odometry',lambda m:odom_cb(m,'lidar'),best),node.create_subscription(Odometry,'/core/state/odometry',lambda m:odom_cb(m,'ekf'),best)]
    def diag_cb(m):store('diagnostic',stamp_ns=ns(m.header.stamp),status=[dict(name=x.name,level=x.level,message=x.message) for x in m.status])
    subs.append(node.create_subscription(DiagnosticArray,'/core/state/lidar_odometry_status',diag_cb,best))
    types=['bool','int8','uint8','int16','uint16','int32','uint32','int64','uint64','float32','float64']
    def def_cb(m,name):definitions[(name,m.definition_hash)]={t:list(getattr(m,t+'_names')) for t in types}
    def val_cb(m,name):
        d=definitions.get((name,m.definition_hash))
        if d is None:return
        values={n:float(v) for t in types for n,v in zip(d[t],getattr(m,t+'_values'))}
        store('timing_debug',node=name,stamp_ns=ns(m.stamp),values=values)
    for name in ['LidarOdometry','StateEstimation']:
        topic='/debug/core/state/'+name
        subs.append(node.create_subscription(TSLDefinition,topic+'/def',lambda m,n=name:def_cb(m,n),best))
        subs.append(node.create_subscription(TSLValues,topic,lambda m,n=name:val_cb(m,n),best))
    static=StaticTransformBroadcaster(node);transforms=[]
    allowed={'front_wheel','imu_bosch','lidar_front','lidar_left','lidar_right'}
    for e in audit['tf_static']:
        if e['child'] not in allowed:continue
        t=TransformStamped();t.header.frame_id=e['parent'];t.child_frame_id=e['child'];stamp(t.header.stamp,now['sim'])
        t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=e['translation']
        t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=e['quaternion'];transforms.append(t)
    static.sendTransform(transforms)
    def clock(n):
        now['sim']=int(n);m=Clock();stamp(m.clock,n);clock_pub.publish(m)
    processes=[];logs=[];reason='COMPLETED';previous=None;started=None
    try:
        clock(now['sim'])
        for binary,name in zip(binaries,['LidarOdometry','StateEstimation']):
            log=(out/(name+'.log')).open('wb');logs.append(log)
            processes.append(subprocess.Popen([str(binary),'--ros-args','-r','__ns:=/core/state','-r','__node:='+name,'--params-file',str(config)],stdout=log,stderr=subprocess.STDOUT))
        # Start-up only, no sensor inputs. Map allocation belongs to startup timing.
        wall=time.monotonic()
        while time.monotonic()-wall<15:
            if any(p.poll() is not None for p in processes):raise RuntimeError('NODE_STARTUP_EXIT')
            if 'Node initialized!' in (out/'LidarOdometry.log').read_text(errors='replace'):break
            time.sleep(.05)
        else:raise RuntimeError('MAP_STARTUP_TIMEOUT')
        time.sleep(1.)
        graph={name:node.get_subscriber_names_and_types_by_node(name,'/core/state') for name in ['LidarOdometry','StateEstimation']}
        (out/'ros_subscriptions.json').write_text(json.dumps(graph,indent=2)+'\n')
        if any(any(s in topic.lower() for s in ['gps','gnss','vectornav','registered']) for topics in graph.values() for topic,_ in topics):raise RuntimeError('REFERENCE_INPUT_IN_GRAPH')
        started=time.monotonic_ns();origin=ev[0][0];origin_wall=started
        for available,kind,e in ev:
            due=origin_wall+(available-origin)
            while time.monotonic_ns()<due:
                clock(origin+time.monotonic_ns()-origin_wall);time.sleep(.001)
            clock(origin+time.monotonic_ns()-origin_wall)
            if any(p.poll() is not None for p in processes):raise RuntimeError('NODE_EXIT_DURING_WINDOW')
            if sum(f.stat().st_size for f in out.iterdir() if f.is_file())>1024**3:raise RuntimeError('RESULT_BUDGET')
            tick=time.monotonic_ns()
            if kind==0:
                if previous is None:
                    previous=dict(stamp_ns=e['header_ns'],omega_base=Tbi[:3,:3]@e['omega']);continue
                z=adapt_imu(e['omega'],e['force'],Tbi,e['header_ns'],previous);previous=z
                m=Imu();stamp(m.header.stamp,e['header_ns']);m.header.frame_id='base_link';m.orientation_covariance[0]=-1.
                m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z=map(float,z['omega_base'])
                m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z=map(float,z['force_base'])
                # Preserve source unavailable covariance marker; no invented IMU R.
                # Native single-IMU weighting clamps then normalizes; Q stays native.
                m.angular_velocity_covariance[0]=-1.;m.linear_acceleration_covariance[0]=-1.
                imu_pub.publish(m)
            else:
                job=e['job'];clouds=[]
                for c in job['clouds']:
                    sid={'lidar_front':0,'lidar_left':1,'lidar_right':2}[c['frame']]
                    payload=(data/Path(c['raw_path']).relative_to('data')).read_bytes()
                    clouds.append(decode_cloud(payload,c['fields'],c['width']*c['height'],c['point_step'],c['header_ns'],c['arrival_ns'],sid,c['message_id'],c['T_body_lidar'],c['bigendian']))
                read_done=time.monotonic_ns();f,meta=fuse_three(clouds);fused_done=time.monotonic_ns()
                m=PointCloud2();stamp(m.header.stamp,meta['header_ns']);m.header.frame_id='base_link';m.height=1;m.width=len(f);m.point_step=f.dtype.itemsize;m.row_step=m.width*m.point_step;m.is_dense=True
                m.fields=[PointField(name=k,offset=f.dtype.fields[k][1],datatype=PointField.FLOAT32 if k!='sensor_id' else PointField.UINT8,count=1) for k in f.dtype.names];m.data=f.tobytes()
                clock(origin+time.monotonic_ns()-origin_wall);cloud_pub.publish(m)
                inputs.append(dict(index=e['index'],stamp_ns=meta['header_ns'],available_ns=available,publish_wall_ns=time.monotonic_ns(),publish_sim_ns=now['sim'],read_decode_ms=(read_done-tick)*1e-6,fusion_ms=(fused_done-read_done)*1e-6,prepare_publish_ms=(time.monotonic_ns()-tick)*1e-6,source_n=len(f)))
        end=time.monotonic()
        while time.monotonic()-end<.5:
            clock(origin+time.monotonic_ns()-origin_wall);time.sleep(.002)
    except Exception as e:reason=str(e)
    finally:
        for p in processes:
            if p.poll() is None:p.send_signal(signal.SIGINT)
        for p in processes:
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:p.kill();p.wait()
        for f in logs:f.close()
        executor.shutdown();thread.join(timeout=2);node.destroy_node();rclpy.shutdown()
        record_stream.close();(out/'inputs.json').write_text(json.dumps(inputs,indent=2)+'\n')
        (out/'run_status.json').write_text(json.dumps(dict(reason=reason,cloud_inputs=len(inputs),records=len(records),process_returncodes=[p.returncode for p in processes],started_wall_ns=started),indent=2)+'\n')
    print(a.window,reason,len(inputs),'clouds',len(records),'output records')
    if reason!='COMPLETED':raise SystemExit(1)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',required=True);p.add_argument('--source',required=True);p.add_argument('--data',required=True);p.add_argument('--attempt',choices=['retry1']);p.add_argument('--window',choices=['normal8','right3'],required=True);main(p.parse_args())
