"""Real ROS services + PTY, mocked hand only. Must run on isolated localhost."""
import importlib.util
import json
import os
from pathlib import Path
import pty
import signal
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
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import Int32
from pose_common import load_pose_set

fake_sdk = MagicMock()
fake_sdk.JointCommand = lambda p,v,e: NS(position=p)
sys.modules['wuji_sdk'] = fake_sdk
base = Path(__file__).parent
spec = importlib.util.spec_from_file_location('driver_test', base.parent / 'examples/python/retargeting/wujihand2_ros_driver.py')
driver = importlib.util.module_from_spec(spec)
spec.loader.exec_module(driver)
events = []
enabled = False
def enable():
    global enabled
    events.append(('enable', None))
    enabled = True
def disable():
    global enabled
    events.append(('disable', None))
    enabled = False
hand = MagicMock()
hand.serial_number = 'SIMULATED_RIGHT_ONLY'
hand.enable.side_effect = enable
hand.disable.side_effect = disable
driver.wait_enabled = lambda h: enabled
driver.wait_disabled = lambda h: not enabled
slot = driver.Hand2Slot.__new__(driver.Hand2Slot)
slot.hand = hand
slot._lock = threading.Lock()
slot._motors_on = False
slot._last_cmd = [9.] * 20
slot._q_filt = None
slot._cmds = [None] * 20
slot._pub = NS(send=lambda cmds: events.append(('target', [q.position for q in cmds])))
slot._node_logger = MagicMock()
slot._feedback_q = [0.] * 20
slot._feedback_seen = time.monotonic()
rclpy.init()
node = rclpy.create_node('manual_teach_fake_driver')
owner = driver.WujiHand2RosDriver.__new__(driver.WujiHand2RosDriver)
owner._node = node
owner._teaching = False
owner._mode = owner._last_mode = 0
owner._teach_slots = {'right': slot}
node.create_service(SetBool, '/tj/hand_right2/teach_hold', owner._on_teach_hold)
node.create_service(Trigger, '/tj/hand_right2/teach_finish', owner._on_teach_finish)
tmp = tempfile.TemporaryDirectory()
path = Path(tmp.name) / 'poses.json'
os.environ['WUJI_RIGHT_SN'] = hand.serial_number
children = []
refresh = True
actual = [0.] * 20
def spawn(*args):
    master, slave = pty.openpty()
    proc = subprocess.Popen([sys.executable, '-u', str(base/'record_poses.py'), '--file', str(path), *args],
                            stdin=slave, stdout=slave, stderr=slave)
    os.close(slave)
    os.set_blocking(master, False)
    child = dict(proc=proc, fd=master, text='')
    children.append(child)
    return child
def tick(seconds):
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        if refresh:
            slot._feedback_q = list(actual)
            slot._feedback_seen = time.monotonic()
        rclpy.spin_once(node, timeout_sec=.01)
        if owner._mode == 0:
            slot.hold_last()
        for child in children:
            try:
                child['text'] += os.read(child['fd'], 65536).decode(errors='replace')
            except (BlockingIOError, OSError):
                pass
def wait(child, text):
    until = time.monotonic()+12
    while text not in child['text'] and time.monotonic() < until:
        tick(.03)
    assert text in child['text'], child['text']
def exited(child, code):
    until=time.monotonic()+3
    while child['proc'].poll() is None and time.monotonic()<until:
        tick(.03)
    assert child['proc'].poll() == code, child['text']
results=[]
try:
    child=spawn()
    wait(child, '手动摆好')
    assert not enabled and slot._last_cmd is None and owner._teaching
    os.write(child['fd'], b'x')
    wait(child, '请先手动摆放并按 B')
    assert not path.exists()
    expected=[]
    for i in range(3):
        actual=[.1*(i+1)+j*.001 for j in range(20)]
        expected.append(list(actual))
        tick(.7)
        start=len(events)
        os.write(child['fd'], b'b')
        tick(.4)
        assert enabled, child['text']
        segment=events[start:]
        on=next(j for j,e in enumerate(segment) if e[0]=='enable')
        assert segment[on-1][0]=='target'
        assert all(abs(a-b)<1e-6 for a,b in zip(segment[on-1][1],expected[-1]))
        # A conflicting mode cannot override the teach hold.
        owner._on_mode(Int32(data=1))
        assert owner._mode==0
        # Actual feedback drifts after enabling; file must retain pre-enable pose.
        actual=[q+.02 for q in actual]
        tick(.3)
        os.write(child['fd'], b'x')
        wait(child, f'已保存手型 {i+1}/3')
        tick(.2)
        assert not enabled and slot._last_cmd is None
    exited(child,0)
    assert not owner._teaching
    assert [p['right'] for p in load_pose_set(path,['right'],{'right':hand.serial_number})]==expected
    results.append('B primes actual pose before enable; X saves pre-enable snapshot and disables, all three slots')
    results.append('Old targets cleared; teaching blocks mode override; X without B rejected')
    original=path.read_bytes()
    child=spawn('--overwrite')
    wait(child,'手动摆好')
    os.write(child['fd'],b'b')
    wait(child,'已在手动摆放位置上使能')
    assert enabled
    child['proc'].send_signal(signal.SIGINT)
    exited(child,0)
    assert not enabled and not owner._teaching
    assert path.read_bytes()==original
    results.append('Ctrl+C while enabled disables hand and leaves previous file untouched')
    child=spawn('--overwrite')
    wait(child,'手动摆好')
    refresh=False
    tick(.7)
    start=len(events)
    os.write(child['fd'],b'b')
    exited(child,1)
    assert not any(e[0]=='enable' for e in events[start:])
    assert not enabled
    results.append('Stale feedback refuses enable and exits with release')
    print(json.dumps({'passed':results,'hardware_used':False},ensure_ascii=False,indent=2))
    (base/'manual_test_results.json').write_text(json.dumps({'passed':results,'hardware_used':False},ensure_ascii=False,indent=2))
finally:
    for child in children:
        if child['proc'].poll() is None:
            child['proc'].kill()
            child['proc'].wait()
        os.close(child['fd'])
    node.destroy_node()
    rclpy.shutdown()
    tmp.cleanup()
