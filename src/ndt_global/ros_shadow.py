"""ROS2 JSON-envelope research adapter. No TF publisher; no production topics.

Run with --events for a bounded transport test, or without it as a subscriber.
The wire schema is independent of restricted vehicle messages.
"""
import argparse,json,time
from pathlib import Path
from collections import Counter
import numpy as np
from ndt_global.shadow import Contract,Candidate,PoseHistory,ShadowCorrector

def execute(args):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    rclpy.init(args=[])
    class Bridge(Node):
        def __init__(self):
            super().__init__('ndt_shadow_research',namespace='/research/fusion',enable_rosout=False)
            self.core=None;self.received=0;self.reasons=Counter();self.published=0
            self.output=self.create_publisher(String,'correction',10)
            self.health=self.create_publisher(String,'status',10)
            self.sub=self.create_subscription(String,'input',self.receive,10)
        def receive(self,msg):
            try:
                e=json.loads(msg.data)
                if e.get('schema_version') != 1:raise ValueError('SCHEMA_VERSION')
                kind=e['kind']
                if kind=='odom':
                    c=Contract(**e['contract'])
                    if self.core is None:self.core=ShadowCorrector(PoseHistory(c))
                    self.core.history.add(e['stamp_ns'],e['pose'],c)
                elif kind=='candidate':
                    if self.core is None:raise ValueError('NO_ODOM')
                    d=e['candidate'];d['contract']=Contract(**d['contract']);d['lidars']=tuple(d['lidars'])
                    c=Candidate(**d);decision=self.core.consume(c,e['now_ns']);self.reasons[decision['reason']]+=1
                    self.health.publish(String(data=json.dumps(decision)))
                    result=self.core.publish(c.stamp_ns)
                    if result['published']:
                        result['pose']=result['pose'].tolist();result['correction']=result['correction'].tolist()
                        self.output.publish(String(data=json.dumps(result)));self.published+=1
                else:raise ValueError('UNKNOWN_KIND')
            except (ValueError,KeyError,TypeError) as ex:self.reasons[str(ex)]+=1
            finally:self.received+=1
    node=Bridge()
    try:
        if args.events:
            publisher=node.create_publisher(String,'input',10);deadline=time.monotonic()+5
            while publisher.get_subscription_count()==0 and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
            if publisher.get_subscription_count()==0:raise RuntimeError('DDS_DISCOVERY_TIMEOUT')
            count=0
            for line in Path(args.events).read_text().splitlines():
                publisher.publish(String(data=line));count+=1;deadline=time.monotonic()+2
                while node.received<count and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.01)
                if node.received!=count:raise RuntimeError('DDS_DELIVERY_TIMEOUT')
            report=dict(started=True,transport='std_msgs/String versioned JSON contract',received=node.received,published=node.published,reasons=dict(node.reasons),tf_published=False,live_sensor_test=False)
            if args.report:Path(args.report).write_text(json.dumps(report,indent=2)+'\n')
            print(json.dumps(report))
        else:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--events');p.add_argument('--report');execute(p.parse_args())
