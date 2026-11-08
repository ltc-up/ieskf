"""Isolated AN ROS2 research adapter; JSON schema1, no production TF."""
import argparse,json,time
from pathlib import Path
from collections import Counter
from .shadow import Contract,PoseHistory
from .global_manager import GlobalManager,PositionObservation

def main(a):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    rclpy.init(args=[])
    class Bridge(Node):
        def __init__(self):
            super().__init__('global_multisource_research',namespace='/research/fusion',enable_rosout=False)
            self.core=None;self.received=0;self.states=Counter();self.reasons=Counter();self.corrections=0
            self.output=self.create_publisher(String,'global_correction',10)
            self.health=self.create_publisher(String,'global_status',10)
            self.sub=self.create_subscription(String,'global_input',self.consume,10)
        def consume(self,msg):
            try:
                e=json.loads(msg.data)
                if e.get('schema_version')!=1:raise ValueError('SCHEMA_VERSION')
                contract=Contract(**e['contract'])
                if self.core is None:self.core=GlobalManager(PoseHistory(contract))
                g=self.core;now=e['now_ns'];kind=e['kind']
                if kind=='odom':g.history.add(e['stamp_ns'],e['pose'],contract)
                elif kind=='ndt':
                    d=g.ndt(e['candidates'],e['classification'],e['stamp_ns'],e['group'],now,contract,converged=e.get('converged'));self.reasons[d['reason']]+=1
                elif kind=='gnss':
                    obs=e['observation'];obs['contract']=Contract(**obs['contract']);d=g.position(PositionObservation(**obs),now);self.reasons[d['reason']]+=1
                elif kind=='output':
                    p=g.publish(e['stamp_ns'],now);self.states[p['state']]+=1
                    self.health.publish(String(data=json.dumps({k:v for k,v in p.items() if k not in ('pose','correction')})))
                    if p['published']:
                        p['pose']=p['pose'].tolist();p['correction']=p['correction'].tolist();self.output.publish(String(data=json.dumps(p)));self.corrections+=1
                else:raise ValueError('UNKNOWN_KIND')
            except (ValueError,KeyError,TypeError) as ex:self.reasons['ERROR:'+str(ex)]+=1
            finally:self.received+=1
    node=Bridge()
    try:
        if a.events:
            pub=node.create_publisher(String,'global_input',10);deadline=time.monotonic()+5
            while not pub.get_subscription_count() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
            if not pub.get_subscription_count():raise RuntimeError('DDS_DISCOVERY_TIMEOUT')
            events=json.loads(Path(a.events).read_text())
            for count,e in enumerate(events,1):
                pub.publish(String(data=json.dumps(e)));deadline=time.monotonic()+2
                while node.received<count and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.01)
                if node.received<count:raise RuntimeError('DDS_DELIVERY_TIMEOUT')
            report=dict(started=True,real_input_transport=True,received=node.received,output_states=dict(node.states),observation_reasons=dict(node.reasons),global_corrections=node.corrections,tf_published=False,live_closed_loop_qualified=False)
            Path(a.report).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
        else:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--events');p.add_argument('--report');main(p.parse_args())
