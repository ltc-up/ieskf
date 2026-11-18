"""Actual 100Hz ROS Pose/health IO; no saved-trajectory replay or 15D claims."""
import json
import threading
import time
import numpy as np
from kinematic_realtime_support import predict_publication


class KinematicOutput:
    def __init__(self,out,start,end,logclass,poses,motion):
        import rclpy
        from std_msgs.msg import String
        self.rclpy=rclpy;self.String=String;self.start_ns=int(start);self.end_ns=int(end);self.poses=poses;self.motion=motion
        self.records=logclass(out/'kinematic_output100.jsonl');self.acks=logclass(out/'kinematic_output100_receipts.jsonl');self.errors=[];self.samples=[];self.received=[];self.latencies=[];self.abort=threading.Event();self.stop=False
        rclpy.init();self.node=rclpy.create_node('kg_s3_kinematic_pose_output');topic='/kg_s3/kinematic_bk/research_pose100';self.pub=self.node.create_publisher(String,topic,100)
        def receive(message):
            now=time.monotonic_ns();v=json.loads(message.data);self.received.append(v['seq']);lat=(now-v['send_wall_ns'])/1e6;self.latencies.append(lat);self.acks.put(dict(seq=v['seq'],received_wall_ns=now,loopback_ms=lat))
        self.sub=self.node.create_subscription(String,topic,receive,100);self.spin=threading.Thread(target=self.spinloop,daemon=True);self.spin.start()
    def spinloop(self):
        while not self.stop:self.rclpy.spin_once(self.node,timeout_sec=.002)
    def start(self,epoch):
        def work():
            try:
                for seq in range((self.end_ns-self.start_ns)//10_000_000+1):
                    if self.abort.is_set():break
                    scheduled=self.start_ns+seq*10_000_000;due=epoch+scheduled-self.start_ns
                    while time.monotonic_ns()<due:
                        if self.abort.is_set():return
                        time.sleep(min(.001,max(0,(due-time.monotonic_ns())/1e9)))
                    wall=time.monotonic_ns();now=self.start_ns+wall-epoch;begin=time.perf_counter_ns()
                    packet=predict_publication(self.poses,self.motion,now,now);compute=(time.perf_counter_ns()-begin)/1e6
                    packet.update(seq=seq,scheduled_ns=scheduled,actual_wall_ns=wall,send_wall_ns=time.monotonic_ns(),output_scope='LOCAL_POSE_HEALTH_ONLY',predictor='KINEMATIC_BK',no_joint_15D_covariance=True)
                    t=time.perf_counter_ns();message=self.String();message.data=json.dumps(packet);serialization=(time.perf_counter_ns()-t)/1e6;t=time.perf_counter_ns();self.pub.publish(message);pub=(time.perf_counter_ns()-t)/1e6
                    late=(wall-due)/1e6;self.records.put(dict(packet=packet,extrapolation_ms=compute,serialization_ms=serialization,publish_API_ms=pub,deadline_lateness_ms=late))
                    self.samples.append(dict(seq=seq,valid=packet['valid'],target_ns=now,anchor_age_ms=None if 'anchor_ref_ns' not in packet else (now-packet['anchor_ref_ns'])/1e6,late_ms=late,compute_ms=compute,serialization_ms=serialization,publish_ms=pub))
            except Exception as exc:self.errors.append(repr(exc))
        self.thread=threading.Thread(target=work,daemon=True);self.thread.start()
    def close(self,abort=False):
        if abort:self.abort.set()
        self.thread.join(10)
        if self.thread.is_alive():raise RuntimeError('KINEMATIC_OUTPUT_NOT_FINISHED')
        time.sleep(.2);self.stop=True;self.spin.join(2);self.node.destroy_node();self.rclpy.shutdown();self.records.close();self.acks.close()
        def stat(values):
            a=np.asarray(values,float);return {} if not len(a) else dict(n=len(a),p50=float(np.median(a)),p95=float(np.percentile(a,95)),p99=float(np.percentile(a,99)),max=float(a.max()))
        return dict(scope='real 100Hz local Pose/health, no 15D P/v/Bias',expected=(self.end_ns-self.start_ns)//10_000_000+1,published=len(self.samples),received=len(self.received),unique_received=len(set(self.received)),missing=sorted(set(x['seq'] for x in self.samples)-set(self.received)),valid_pose=sum(x['valid'] for x in self.samples),invalid_health_only=sum(not x['valid'] for x in self.samples),errors=self.errors,aborted=abort,loopback_ms=stat(self.latencies),deadline_lateness_ms=stat([x['late_ms'] for x in self.samples]),anchor_age_ms=stat([x['anchor_age_ms'] for x in self.samples if x['anchor_age_ms'] is not None]),extrapolation_ms=stat([x['compute_ms'] for x in self.samples]),serialization_ms=stat([x['serialization_ms'] for x in self.samples]),publish_API_ms=stat([x['publish_ms'] for x in self.samples]),late_slots_over_10ms=sum(x['late_ms']>10 for x in self.samples),pose_time_reversal=any(b['target_ns']<a['target_ns'] for a,b in zip(self.samples,self.samples[1:])))
