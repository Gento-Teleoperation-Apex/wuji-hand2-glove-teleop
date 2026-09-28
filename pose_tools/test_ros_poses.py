"""Real ROS + PTY integration, isolated localhost domain; no SDK/hardware."""
import json
import os
from pathlib import Path
import pty
import signal
import subprocess
import sys
import tempfile
import time

assert os.environ.get('ROS_DOMAIN_ID') == '224'
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
assert os.environ.get('RMW_IMPLEMENTATION') == 'rmw_fastrtps_cpp'
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from pose_common import JOINT_NAMES, STATE_TOPIC, COMMAND_TOPIC, MODE_TOPIC, load_poses, positions, atomic_save

base = Path(__file__).parent
tmp = tempfile.TemporaryDirectory(prefix='wuji-poses-test-')
path = Path(tmp.name) / 'poses.json'
os.environ['WUJI_RIGHT_SN'] = 'SIMULATED_RIGHT_ONLY'
rclpy.init()
node = rclpy.create_node('simulated_pose_test_no_hardware')
state_pub = node.create_publisher(JointState, STATE_TOPIC, 10)
commands, modes = [], []
node.create_subscription(JointState, COMMAND_TOPIC, lambda m: commands.append(list(m.position)), qos_profile_sensor_data)
node.create_subscription(Int32, MODE_TOPIC, lambda m: modes.append(m.data), 10)
children = []
state = [.1] * 20
send_state = True

def spawn(script, *args):
    master, slave = pty.openpty()
    proc = subprocess.Popen([sys.executable, '-u', str(base / script), '--file', str(path), *args],
                            stdin=slave, stdout=slave, stderr=slave)
    os.close(slave)
    os.set_blocking(master, False)
    child = {'proc': proc, 'fd': master, 'text': ''}
    children.append(child)
    return child

def tick(seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if send_state:
            msg = JointState()
            msg.name = list(reversed(JOINT_NAMES))
            msg.position = list(reversed(state))
            msg.header.stamp = node.get_clock().now().to_msg()
            state_pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=.005)
        for child in children:
            try:
                child['text'] += os.read(child['fd'], 65536).decode(errors='replace')
            except (BlockingIOError, OSError):
                pass
        time.sleep(.01)

def wait_for(child, text, seconds=10):
    deadline = time.monotonic() + seconds
    while text not in child['text'] and time.monotonic() < deadline:
        tick(.05)
    assert text in child['text'], child['text']

def exited(child, code):
    deadline = time.monotonic() + 3
    while child['proc'].poll() is None and time.monotonic() < deadline:
        tick(.05)
    assert child['proc'].poll() == code, child['text']

results = []
try:
    # Manual B/X recording is covered by test_manual_teach.py.
    atomic_save(path, dict(schema_version=1, side='right', units='rad',
        hand_serial=os.environ['WUJI_RIGHT_SN'], joint_names=JOINT_NAMES,
        poses=[dict(positions=[slot*.1+i*.001 for i in range(20)]) for slot in range(1,4)]))
    saved = load_poses(path, os.environ['WUJI_RIGHT_SN'])
    assert saved == [[slot * .1 + i * .001 for i in range(20)] for slot in range(1, 4)]
    repeat = spawn('record_poses.py')
    exited(repeat, 2)
    assert load_poses(path, os.environ['WUJI_RIGHT_SN']) == saved
    results.append('Existing recording protected without --overwrite')

    player = spawn('play_poses.py', '--transition', '.2')
    wait_for(player, '当前手型 1/3')
    tick(1)
    assert commands and all(abs(a-b) < 1e-8 for a,b in zip(commands[-1], saved[0]))
    assert 3 in modes
    for expected in [2, 0, 1, 1]:
        os.write(player['fd'], str(expected + 1).encode())
        tick(.9)
        assert all(abs(a-b) < 1e-8 for a,b in zip(commands[-1], saved[expected])), player['text']
    count = len(commands)
    tick(.5)
    assert len(commands) > count + 10
    os.write(player['fd'], b'H')
    tick(.3)
    assert all(abs(a-b) < 1e-8 for a,b in zip(commands[-1], saved[1]))
    results.append('1/2/3 select exact pose in arbitrary order; repeated key holds; H ignored; user mode=3')
    send_state = False
    tick(1.5)
    exited(player, 1)
    assert modes[-1] == 0
    results.append('Stale feedback stops player and publishes standby mode=0')
    send_state = True
    tick(.5)
    blocker = node.create_publisher(Int32, MODE_TOPIC, 10)
    count = len(commands)
    conflict = spawn('play_poses.py')
    wait_for(conflict, '存在其他模式/手型发布端')
    exited(conflict, 1)
    assert len(commands) == count
    node.destroy_publisher(blocker)
    results.append('Existing mode owner rejected without motion commands')
    tick(.5)
    normal_exit = spawn('play_poses.py', '--transition', '.2')
    wait_for(normal_exit, '当前手型 1/3')
    tick(.4)
    os.write(normal_exit['fd'], b'q')
    exited(normal_exit, 0)
    assert modes[-1] == 0
    tick(.5)
    interrupted = spawn('play_poses.py', '--transition', '.2')
    wait_for(interrupted, '当前手型 1/3')
    tick(.4)
    interrupted['proc'].send_signal(signal.SIGINT)
    exited(interrupted, 0)
    assert modes[-1] == 0
    assert 'Traceback' not in interrupted['text'], interrupted['text']
    results.append('Q and Ctrl+C exit cleanly and request standby holding')
    try:
        load_poses(path, 'WRONG_SERIAL')
    except ValueError:
        pass
    else:
        raise AssertionError('wrong serial accepted')
    try:
        positions(JOINT_NAMES, [float('nan')] * 20)
    except ValueError:
        pass
    else:
        raise AssertionError('NaN accepted')
    results.append('Wrong hand serial and nonfinite angles rejected')
    print(json.dumps({'passed': results, 'hardware_used': False}, indent=2))
    (base / 'test_results.json').write_text(json.dumps({'passed': results, 'hardware_used': False}, indent=2))
finally:
    for child in children:
        if child['proc'].poll() is None:
            child['proc'].terminate()
            try:
                child['proc'].wait(timeout=3)
            except subprocess.TimeoutExpired:
                child['proc'].kill()
                child['proc'].wait()
        os.close(child['fd'])
    node.destroy_node()
    rclpy.shutdown()
    tmp.cleanup()
