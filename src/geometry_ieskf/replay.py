from .inertial import *
from .wheel import *
from .iterated import *
from .diagnostics import *
import time,collections,os
class EventReplay:
 """Deterministic physical/Header-time sorting. Not an online asynchronous scheduler."""
 def __init__(self,contract,geometry=True,source_loader=None):
  self.source_loader=source_loader;self.contract=contract;self.geometry_enabled=geometry;self.lever=np.array(contract['extrinsic']['lever_body_imu_m']);self.factor=GeometryFactor(self.lever);self.predictor=InertialPredictor();self.wheel=WheelObservation(self.lever);self.solver=IteratedErrorStateUpdate(self.factor);self.diagnostics=Diagnostics()
 def run(self,data,mapquery):
  diag=self.diagnostics;start=time.perf_counter();es=[];guards=[];seen_stamps=set();last_recorded={}
  for event in data['events']:
   if event['kind'] in ['imu','wheel']:
    key=(event['topic'],event['header_ns'])
    if not event['valid']:
     guards.append({'message_id':event['message_id'],'reason':'INVALID_SENSOR_EVENT_REJECTED'});continue
    if key in seen_stamps:
     guards.append({'message_id':event['message_id'],'reason':'DUPLICATE_HEADER_REJECTED'});continue
    if event['topic'] in last_recorded and event['header_ns']<last_recorded[event['topic']]:
     guards.append({'message_id':event['message_id'],'reason':'RECORDER_ORDER_ROLLBACK_OFFLINE_REORDERED'})
    seen_stamps.add(key);last_recorded[event['topic']]=event['header_ns']
   es.append(event)
  before=[e for e in es if e['kind']=='initial_pose' and e['valid'] and e['header_ns']<=data['begin_ns']];init=max(before,key=lambda e:e['header_ns']);t0=init['header_ns']
  imu=max((e for e in es if e['kind']=='imu' and e['valid'] and e['header_ns']<=t0),key=lambda e:e['header_ns']);wh=max((e for e in es if e['kind']=='wheel' and e['valid'] and e['header_ns']<=t0),key=lambda e:e['header_ns'])
  assert (t0-wh['header_ns'])*1e-9<=C['wheel_max_age_s'] and (t0-imu['header_ns'])*1e-9<=C['max_imu_interval_s']
  R=Rotation.from_quat(init['quaternion_xyzw']).as_matrix();x=State(np.array(init['position'])+R@self.lever,R@(np.array([wh['speed'],0.,0.])+np.cross(imu['omega'],self.lever)),R,np.zeros(3),np.zeros(3),t0)
  P=np.diag(np.repeat(C['initial_std_p_v_theta_bg_ba'],3)**2);initial=diag.snapshot(x,P);initial.update(source='historical fused pose only at initialization; wheel longitudinal velocity; uncalibrated zero bias mean',pose_message_id=init['message_id'],wheel_message_id=wh['message_id'],imu_message_id=imu['message_id'])
  events=[e for e in es if e['kind'] in ['imu','wheel'] and t0<e['header_ns']<=data['end_ns']]+data['frames']
  events +=[{'kind':'output','header_ns':t} for t in range(t0+10_000_000,data['end_ns']+1,10_000_000)]
  priority={'imu':0,'wheel':1,'lidar':2,'output':3};events.sort(key=lambda e:(e['header_ns'],priority[e['kind']],e.get('message_id',0)))
  outputs=[];lidar=[];wheel=[];predictions=[];counts=collections.Counter();used=set();last_imu=imu;aborted=None
  for e in events:
   if not e.get('valid',True):counts[e['kind']+'_invalid']+=1;continue
   t=e['header_ns'];dt=(t-x.t)*1e-9
   if dt<0:raise RuntimeError('STATE_TIME_ROLLBACK')
   if (t-last_imu['header_ns'])*1e-9>C['max_imu_interval_s']:
    aborted={'reason':'IMU_GAP_STOP_SEGMENT','event_time_ns':t,'last_imu_ns':last_imu['header_ns']};break
   if dt>0:
    tick=time.perf_counter();x,P=self.predictor.predict(x,P,np.array(last_imu['omega']),np.array(last_imu['force']),dt);x.t=t;diag.timing('inertial_prediction',1000*(time.perf_counter()-tick));counts['propagation_subintervals']+=1
   if e['kind']=='imu':last_imu=e;counts['imu']+=1
   elif e['kind']=='wheel':
    tick=time.perf_counter();x,P,info=self.wheel.correct(x,P,np.array(last_imu['omega']),e['speed'],e['variance']);diag.timing('wheel_update',1000*(time.perf_counter()-tick));wheel.append({'time_ns':t,'message_id':e['message_id'],**info});counts['wheel_accepted' if info['accepted'] else 'wheel_rejected']+=1
   elif e['kind']=='lidar':
    if e['source_sha256'] in used:raise RuntimeError('DUPLICATE_LIDAR_INFORMATION')
    used.add(e['source_sha256']);tick=time.perf_counter();source=self.source_loader(e) if self.source_loader else np.load(Path(os.environ['IESKF_DATA_ROOT'])/e['source_file']);diag.timing('source_file_load',1000*(time.perf_counter()-tick));prior=x.copy();priorP=P.copy();pre=diag.snapshot(x,P)
    if self.geometry_enabled:
     x,P,info=self.solver.correct(x,P,source,mapquery)
    else:info={'accepted':False,'reason':'INERTIAL_WHEEL_CONTROL','iterations':[],'wall_ms':0.}
    diag.timing('lidar_update',info['wall_ms']);counts['lidar_accepted' if info['accepted'] else 'lidar_rejected']+=1
    correction=difference(x,prior);bodydelta=np.r_[prior.R.T@correction[:3],correction[6:9]]
    # At IMU origin; diagnostic base-link translation is computed separately.
    basebefore=self.factor.transform(prior);baseafter=self.factor.transform(x);basebody=prior.R.T@(baseafter[:3,3]-basebefore[:3,3])
    lidar.append({'header_ns':t,'source_sha256':e['source_sha256'],'historical_common_header_ns':e['common_header_ns'],'predicted':pre,'corrected':diag.snapshot(x,P),'base_body_translation_correction_m':basebody.tolist(),'body_pose_delta_imu':bodydelta.tolist(),'information_prior_P':priorP.tolist(),'state_latency_ns':x.t-t,**info})
   elif e['kind']=='output':
    s=diag.snapshot(x,P);s['p_base_map']=self.factor.transform(x)[:3,3].tolist();s['output_time_ns']=t;outputs.append(s);counts['output']+=1
   if e['kind']=='imu':predictions.append({'header_ns':t,**diag.snapshot(x,P)})
  wall=time.perf_counter()-start
  return {'name':data['name'],'initialization':initial,'counts':dict(counts),'aborted':aborted,'elapsed_sensor_s':(x.t-t0)*1e-9,'cpu_replay_wall_s':wall,'offline_replay_sensor_seconds_per_wall_second':(x.t-t0)*1e-9/max(wall,1e-10),'outputs':outputs,'imu_states':predictions,'lidar_updates':lidar,'wheel_updates':wheel,'diagnostics':diag.summary(),'output_expected_count':(data['end_ns']-t0)//10_000_000,'unique_lidar_sources':len(used),'no_ndt_input':True,'geometry_enabled':self.geometry_enabled,'event_guards':guards}
