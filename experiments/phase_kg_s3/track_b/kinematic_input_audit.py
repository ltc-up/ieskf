"""Complete raw sensor HEADER/availability metadata audit; no motion values out.

Future metadata establishes coverage/gap and retry event boundaries only. Actual
measurement values must still enter through recorder-order continuous_inputs.
"""
import hashlib
import json
import sqlite3
import numpy as np
from kinematic_predictor import HeaderCoverageAudit


def build_header_index(bag,start_ns,end_ns):
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu
    from geometry_msgs.msg import TwistWithCovarianceStamped
    db=sqlite3.connect('file:'+str(bag)+'?mode=ro&immutable=1',uri=True)
    topics=dict(db.execute('SELECT id,name FROM topics'))
    kinds={i:('imu' if n=='/sensor/bosch/imu' else 'wheel') for i,n in topics.items() if n in ('/sensor/bosch/imu','/localization/wheel_velocity')}
    records=[];h=hashlib.sha256();headers={'imu':[],'wheel':[]};last=None
    frames={'imu':set(),'wheel':set()};counts={'imu':0,'wheel':0};wheel_variance={'finite_nonnegative':0,'nonfinite':0,'negative':0}
    query='SELECT id,topic_id,timestamp,data FROM messages WHERE timestamp>=? AND topic_id IN ('+','.join('?' for _ in kinds)+') ORDER BY timestamp,id'
    for ident,tid,available,blob in db.execute(query,[int(start_ns),*kinds]):
        kind=kinds[tid];m=deserialize_message(blob,Imu if kind=='imu' else TwistWithCovarianceStamped)
        stamp=int(m.header.stamp.sec)*10**9+int(m.header.stamp.nanosec)
        # Only metadata/variance validity leaves this routine; gyro/speed never do.
        frames[kind].add(str(m.header.frame_id));counts[kind]+=1
        if kind=='wheel':
            variance=float(m.twist.covariance[0])
            wheel_variance['nonfinite' if not np.isfinite(variance) else ('negative' if variance<0 else 'finite_nonnegative')]+=1
        row=dict(kind=kind,header_ns=stamp,available_ns=available,message_id=ident,cdr_sha256=hashlib.sha256(blob).hexdigest())
        h.update(json.dumps(row,sort_keys=True).encode());last=available
        if stamp<=end_ns:
            headers[kind].append(stamp);records.append(row)
    db.close()
    audit=HeaderCoverageAudit(headers['imu'],headers['wheel'],int(start_ns),int(end_ns),h.hexdigest())
    metadata=dict(kind='complete original header and availability audit; no future motion values',identity=h.hexdigest(),covers_from_ns=int(start_ns),covers_to_ns=int(end_ns),original_sensor_index_examined_to_available_ns=last,minimum_complete_motion_available_end_ns=max([int(end_ns)]+[r['available_ns'] for r in records]),records=records,original_frame_ids={k:sorted(v) for k,v in frames.items()},examined_message_counts=counts,wheel_variance_validity=wheel_variance,frame_semantics='raw original header frame; IMU rotation is applied once by continuous_inputs, tail reader applies same one rotation')
    return audit,metadata


def released_tail_motion(bag,metadata,after_available_ns,R_base_imu):
    """Original motion CDR at real release times; no extra LiDAR or A0 state."""
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu
    from geometry_msgs.msg import TwistWithCovarianceStamped
    db=sqlite3.connect('file:'+str(bag)+'?mode=ro&immutable=1',uri=True)
    try:
        for row in metadata['records']:
            if row['available_ns']<=after_available_ns:continue
            raw=db.execute('SELECT timestamp,data FROM messages WHERE id=?',(row['message_id'],)).fetchone()
            if raw is None or raw[0]!=row['available_ns'] or hashlib.sha256(raw[1]).hexdigest()!=row['cdr_sha256']:raise ValueError('TAIL_RAW_IDENTITY_MISMATCH')
            kind=row['kind'];m=deserialize_message(raw[1],Imu if kind=='imu' else TwistWithCovarianceStamped)
            header=int(m.header.stamp.sec)*10**9+int(m.header.stamp.nanosec)
            if header!=row['header_ns']:raise ValueError('TAIL_RAW_HEADER_MISMATCH')
            event=dict(kind=kind,header_ns=header,available_ns=row['available_ns'],message_id=row['message_id'])
            if kind=='imu':event['omega_base']=(R_base_imu@np.array([m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z])).tolist()
            else:event.update(speed_base=m.twist.twist.linear.x,variance=m.twist.covariance[0])
            yield event,row
    finally:db.close()


def verify_frame_binding(metadata,static_audit,T_base_imu):
    """Fail rather than silently reinterpret an unexpected measurement frame."""
    from scipy.spatial.transform import Rotation
    expected={'imu':['imu_bosch'],'wheel':['base_link']}
    if metadata['original_frame_ids']!=expected:raise ValueError('RAW_MOTION_FRAME_CONTRACT_MISMATCH')
    edges={e['child']:e for e in static_audit['tf_static']}
    def transform(child,seen=()):
        if child=='base_link':return np.eye(4)
        if child in seen:raise ValueError('STATIC_FRAME_CYCLE')
        e=edges[child];T=np.eye(4);T[:3,:3]=Rotation.from_quat(e['quaternion']).as_matrix();T[:3,3]=e['translation']
        return transform(e['parent'],seen+(child,))@T
    residual=float(np.max(np.abs(transform('imu_bosch')-np.asarray(T_base_imu))))
    if not np.isfinite(residual) or residual>1e-12:raise ValueError('IMU_STATIC_CHAIN_BINDING_MISMATCH')
    return dict(raw_frames=expected,T_base_imu_static_chain_maxabs=residual,gyro_rotation_count=1,wheel_linear_x_units='m/s, original production wheel model and recorded topic; no new scaling',wheel_repeated_lever_correction=False)
