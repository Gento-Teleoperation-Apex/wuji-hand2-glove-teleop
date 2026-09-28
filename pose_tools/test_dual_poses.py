"""Both-hand ROS/PTTY workflow with fake SDK hardware; no physical devices."""
import importlib.util
import json
import os
from pathlib import Path
import pty
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace as NS
from unittest.mock import MagicMock

assert os.environ.get('ROS_DOMAIN_ID') == '224'
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from std_srvs.srv import SetBool, Trigger
from rclpy.qos import qos_profile_sensor_data
from pose_common import JOINT_NAMES, MODE_TOPIC, command_topic, state_topic, load_pose_set

sdk = MagicMock()
sdk.JointCommand = lambda p,v,e: NS(position=p)
sys.modules['wuji_sdk'] = sdk
base = Path(__file__).parent
spec = importlib.util.spec_from_file_location('dual_driver_test', base.parent/'examples/python/retargeting/wujihand2_ros_driver.py')
driver = importlib.util.module_from_spec(spec)
spec.loader.exec_module(driver)
driver.wait_enabled = lambda h: h.enabled
driver.wait_disabled = lambda h: not h.enabled
sides = ['left','right']
serials = {s:f'SIMULATED_{s.upper()}' for s in sides}
for s in sides:
    os.environ[f'WUJI_{s.upper()}_SN'] = serials[s]
events = []
actual = {s:[0.]*20 for s in sides}
slots = {}
def make_slot(side):
    hand = MagicMock()
    hand.serial_number = serials[side]
    hand.enabled = False
    hand.fail_enable = False
    def enable():
        events.append((side,'enable',None))
        if hand.fail_enable:
            raise RuntimeError('simulated enable failure')
        hand.enabled = True
    def disable():
        hand.enabled=False
        events.append((side,'disable',None))
    hand.enable.side_effect=enable
    hand.disable.side_effect=disable
    slot=driver.Hand2Slot.__new__(driver.Hand2Slot)
    slot.hand=hand
    slot._lock=threading.Lock()
    slot._motors_on=False
    slot._last_cmd=[9.]*20
    slot._q_filt=None
    slot._cmds=[None]*20
    slot._pub=NS(send=lambda q: events.append((side,'target',[x.position for x in q])))
    slot._node_logger=MagicMock()
    slot._feedback_q=[0.]*20
    slot._feedback_seen=time.monotonic()
    return slot
slots={s:make_slot(s) for s in sides}
rclpy.init()
node=rclpy.create_node('dual_pose_fake_driver')
owner=driver.WujiHand2RosDriver.__new__(driver.WujiHand2RosDriver)
owner._node=node
owner._teaching=False
owner._mode=owner._last_mode=0
owner._home_available=False
owner._teach_slots=slots
node.create_service(SetBool,'/tj/hands2/teach_hold',owner._on_teach_hold)
node.create_service(Trigger,'/tj/hands2/teach_finish',owner._on_teach_finish)
pubs={s:node.create_publisher(JointState,state_topic(s),10) for s in sides}
commands={s:[] for s in sides}
modes=[]
def command(side,msg):
    commands[side].append((list(msg.position),(msg.header.stamp.sec,msg.header.stamp.nanosec)))
    if owner._mode==3:
        actual[side]=list(msg.position)
def mode(msg):
    modes.append(msg.data)
    owner._on_mode(msg)
node.create_subscription(Int32,MODE_TOPIC,mode,10)
for s in sides:
    node.create_subscription(JointState,command_topic(s),lambda m,s=s:command(s,m),qos_profile_sensor_data)
tmp=tempfile.TemporaryDirectory()
path=Path(tmp.name)/'both.json'
children=[]
send_sides=set(sides)
def spawn(script,*args):
    master,slave=pty.openpty()
    proc=subprocess.Popen([sys.executable,'-u',str(base/script),'--side','both','--file',str(path),*args],
                          stdin=slave,stdout=slave,stderr=slave)
    os.close(slave)
    os.set_blocking(master,False)
    child=dict(proc=proc,fd=master,text='')
    children.append(child)
    return child
def tick(seconds):
    until=time.monotonic()+seconds
    while time.monotonic()<until:
        for s in send_sides:
            slots[s]._feedback_q=list(actual[s])
            slots[s]._feedback_seen=time.monotonic()
            m=JointState(name=JOINT_NAMES,position=actual[s])
            m.header.stamp=node.get_clock().now().to_msg()
            pubs[s].publish(m)
        rclpy.spin_once(node,timeout_sec=.005)
        for c in children:
            try:
                c['text']+=os.read(c['fd'],65536).decode(errors='replace')
            except (BlockingIOError,OSError):
                pass
        time.sleep(.005)
def wait(c,text):
    until=time.monotonic()+12
    while text not in c['text'] and time.monotonic()<until:
        tick(.04)
    assert text in c['text'],c['text']
def exited(c,code):
    until=time.monotonic()+4
    while c['proc'].poll() is None and time.monotonic()<until:
        tick(.04)
    assert c['proc'].poll()==code,c['text']
results=[]
try:
    c=spawn('record_poses.py')
    wait(c,'手动摆好')
    assert all(not x.hand.enabled and x._last_cmd is None for x in slots.values())
    expected=[]
    for i in range(3):
        actual={s:[(-1 if s=='left' else 1)*(.1*(i+1)+j*.001) for j in range(20)] for s in sides}
        expected.append({s:list(q) for s,q in actual.items()})
        tick(.7)
        os.write(c['fd'],b'b')
        tick(.3)
        assert all(x.hand.enabled for x in slots.values()),c['text']
        actual={s:[v+.01 for v in q] for s,q in actual.items()}
        tick(.4)
        os.write(c['fd'],b'x')
        wait(c,f'已保存手型 {i+1}/3')
        tick(.2)
        assert all(not x.hand.enabled for x in slots.values())
    exited(c,0)
    assert load_pose_set(path,sides,serials)==expected
    results.append('B/X stores three paired 40-joint snapshots before enable, then disables both')
    tick(.3)
    c=spawn('play_poses.py','--transition','.2')
    wait(c,'当前手型 1/3')
    selected = {'left': 0, 'right': 0}
    for key, side, index in [('3','right',2), ('S','left',1), ('1','right',0),
                              ('d','left',2), ('2','right',1), ('a','left',0), ('a','left',0)]:
        os.write(c['fd'],key.encode())
        selected[side] = index
        tick(.7)
        for s in sides:
            assert all(abs(a-b)<1e-8 for a,b in zip(commands[s][-1][0],expected[selected[s]][s])),c['text']
        # The executor may have delivered only one side of the newest pair.
        stamps = [{stamp for _,stamp in commands[s]} for s in sides]
        assert len(stamps[0] & stamps[1]) >= 5
    results.append('Right 1/2/3 and left A/S/D select independently; other hand holds; case and repeat checked')
    send_sides={'right'}
    tick(1.4)
    exited(c,1)
    assert modes[-1]==0
    results.append('Losing either hand feedback stops the entire dual pose player')
    send_sides=set(sides)
    tick(.5)
    c=spawn('record_poses.py','--overwrite')
    wait(c,'手动摆好')
    slots['right'].hand.fail_enable=True
    os.write(c['fd'],b'b')
    exited(c,1)
    assert all(not x.hand.enabled and x._last_cmd is None for x in slots.values())
    assert load_pose_set(path,sides,serials)==expected
    results.append('Failure enabling second hand rolls back both; old recordings preserved')
    results.append('No real SDK imported or hardware used')
    report={'passed':results,'hardware_used':False}
    print(json.dumps(report,indent=2))
    (base/'dual_test_results.json').write_text(json.dumps(report,indent=2))
finally:
    for c in children:
        if c['proc'].poll() is None:
            c['proc'].kill()
            c['proc'].wait()
        os.close(c['fd'])
    node.destroy_node()
    rclpy.shutdown()
    tmp.cleanup()
