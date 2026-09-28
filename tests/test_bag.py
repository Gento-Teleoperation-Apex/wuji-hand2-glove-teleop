"""Real ROS/MCAP subprocess test with synthetic feedback and no SDK imports."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
assert os.environ.get('ROS_DOMAIN_ID')=='226' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
import yaml
root=Path(__file__).resolve().parents[1]
rclpy.init();node=rclpy.create_node('fake_bag_hand_driver');sides=['left','right']
pubs={s:node.create_publisher(JointState,f'/tj/hand_{s}2/joint_states',qos_profile_sensor_data) for s in sides}
received={s:[] for s in sides};modes=[]
for s in sides:node.create_subscription(JointState,f'/tj/hand_{s}_replay',lambda m,s=s:received[s].append(list(m.position)),qos_profile_sensor_data)
node.create_subscription(Int32,'/tj/control/footkey2',lambda m:modes.append(m.data),10)
def tick(seconds):
    until=time.monotonic()+seconds
    while time.monotonic()<until:
        for s in sides:
            msg=JointState(name=[f'j{i}' for i in range(20)],position=[.1 if s=='right' else -.2]*20);msg.header.stamp=node.get_clock().now().to_msg();pubs[s].publish(msg)
        rclpy.spin_once(node,timeout_sec=.01);time.sleep(.01)
with tempfile.TemporaryDirectory() as tmp:
    bag=Path(tmp)/'sample';log=open(Path(tmp)/'log','w+');child=None
    try:
        child=subprocess.Popen([sys.executable,str(root/'tools/hand_bag.py'),'record',str(bag)],stdout=log,stderr=log)
        tick(8);child.send_signal(signal.SIGINT);child.wait(timeout=15)
        assert child.returncode==0,child.returncode
        meta=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
        assert meta['storage_identifier']=='mcap'
        names={t['topic_metadata']['name'] for t in meta['topics_with_message_count'] if t['message_count']>0}
        assert all(f'/tj/hand_{s}2/joint_states' in names for s in sides)
        child=subprocess.Popen([sys.executable,str(root/'tools/hand_bag.py'),'play',str(bag)],stdout=log,stderr=log)
        until=time.monotonic()+15
        while child.poll() is None and time.monotonic()<until:tick(.1)
        assert child.poll()==0,child.poll();tick(.2)
        assert modes and 4 in modes and modes[-1]==0,modes
        assert all(received[s] for s in sides),received
        assert all(abs(received[s][-1][0]-(.1 if s=='right' else -.2))<1e-6 for s in sides)
        print('PASS: synthetic dual MCAP recording; feedback-only replay to mode-4 channels; standby on completion')
    except BaseException:
        log.flush();log.seek(0);print(log.read());raise
    finally:
        if child is not None and child.poll() is None:child.terminate();child.wait(timeout=10)
        log.close()
node.destroy_node();rclpy.shutdown()
