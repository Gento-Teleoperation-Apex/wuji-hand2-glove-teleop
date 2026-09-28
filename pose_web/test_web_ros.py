"""Isolated ROS/HTTP test, synthetic feedback only, no SDK/hardware."""
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import time
import urllib.request
import urllib.error
assert os.environ.get('ROS_DOMAIN_ID')=='225'
assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from rclpy.qos import qos_profile_sensor_data
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'pose_tools'))
from pose_common import JOINT_NAMES, MODE_TOPIC, state_topic, command_topic, atomic_save
rclpy.init()
node=rclpy.create_node('web_fake_driver_no_hardware')
side_names=['left','right'];serials={s:'FAKE_'+s for s in side_names}
for s in side_names:os.environ['WUJI_'+s.upper()+'_SN']=serials[s]
poses=[{s:[(.1*(i+1))*(-1 if s=='left' else 1)]*20 for s in side_names} for i in range(3)]
tmp=tempfile.TemporaryDirectory();path=Path(tmp.name)/'poses.json'
atomic_save(path,dict(schema_version=2,sides=side_names,hand_serials=serials,units='rad',joint_names=JOINT_NAMES,poses=[dict(positions=q) for q in poses]))
pubs={s:node.create_publisher(JointState,state_topic(s),10) for s in side_names}
commands={s:[] for s in side_names};modes=[]
for s in side_names:node.create_subscription(JointState,command_topic(s),lambda m,s=s:commands[s].append(list(m.position)),qos_profile_sensor_data)
node.create_subscription(Int32,MODE_TOPIC,lambda m:modes.append(m.data),10)
base='http://127.0.0.1:8767';client='test-client-one'
log=open(Path(__file__).parent/'ros_test_server.log','w')
proc=subprocess.Popen([sys.executable,str(Path(__file__).parent/'server.py'),'--host','127.0.0.1','--port','8767','--transition','.2','--file',str(path)],stdout=log,stderr=log)
def tick(seconds):
    until=time.monotonic()+seconds
    while time.monotonic()<until:
        for s in side_names:pubs[s].publish(JointState(name=JOINT_NAMES,position=[0.]*20))
        rclpy.spin_once(node,timeout_sec=.004)
        time.sleep(.005)
def get():
    return json.load(urllib.request.urlopen(base+'/api/status',timeout=2))
def post(action,**extra):
    data=dict(action=action,client=client)
    data.update(extra)
    req=urllib.request.Request(base+'/api/action',data=json.dumps(data).encode(),headers={'Content-Type':'application/json','X-Control-Token':token})
    return json.load(urllib.request.urlopen(req,timeout=3))
try:
    for _ in range(60):
        tick(.1)
        try:
            html=urllib.request.urlopen(base,timeout=.2).read().decode();break
        except (OSError,urllib.error.URLError):pass
    else:raise AssertionError('server did not start')
    token=re.search("const TOKEN='([^']+)'",html).group(1)
    tick(1)
    assert get()['ready'] and not get()['active']
    assert node.count_publishers(MODE_TOPIC)==0 and all(not v for v in commands.values())
    post('start');tick(.5)
    assert modes[-1]==3 and commands['left'][-1]==[0.]*20 and commands['right'][-1]==[0.]*20
    post('select',key='3');tick(.4)
    assert commands['right'][-1]==poses[2]['right'] and commands['left'][-1]==[0.]*20
    post('select',key='S');tick(.4)
    assert commands['left'][-1]==poses[1]['left'] and commands['right'][-1]==poses[2]['right']
    try:post('select',key='1',client='other-client')
    except urllib.error.HTTPError as e:assert e.code==409
    else:raise AssertionError('non-owner accepted')
    post('pause');tick(.3)
    assert modes[-1]==0 and not get()['active'] and node.count_publishers(MODE_TOPIC)==0
    post('start');tick(3.6)
    assert not get()['active'] and modes[-1]==0
    # Other running command-line player blocks web activation.
    blocker=node.create_publisher(Int32,MODE_TOPIC,10);tick(.6)
    assert not get()['ready']
    try:post('start')
    except urllib.error.HTTPError as e:assert e.code==409
    else:raise AssertionError('conflicting player accepted')
    print('PASS: no publishers before enable; current-pose startup; independent ROS targets; owner lock; pause; heartbeat timeout; CLI conflict.')
finally:
    proc.terminate();proc.wait(timeout=8);log.close();node.destroy_node();rclpy.shutdown();tmp.cleanup()
