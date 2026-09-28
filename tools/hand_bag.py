#!/usr/bin/env python3
"""Record dual hand feedback/commands; replay only hand feedback to mode-4 inputs."""
import argparse
from pathlib import Path
import signal
import subprocess
import time


def topics(sides):
    result=[]
    for s in sides:
        result += [f'/tj/hand_{s}2/joint_states',f'/tj/hand_{s}2/joint_commands',f'/tj/hand_{s}_user',f'/tj/hand_{s}_replay',f'/tj/hand_{s}2/tactile']
        result += [f'/tj/hand_{s}2/tactile/{f}' for f in ['thumb','index','middle','ring','pinky']]
    return result+['/tj/control/footkey2']


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=['record','play'])
    p.add_argument('bag',type=Path)
    p.add_argument('--side',choices=['left','right','both'],default='both')
    p.add_argument('--rate',type=float,default=1.)
    args=p.parse_args();sides=['left','right'] if args.side=='both' else [args.side]
    if not .05<=args.rate<=2:p.error('rate must be 0.05..2')
    if args.action=='record':
        if args.bag.exists():p.error('Output already exists; choose a new directory')
        child=subprocess.Popen(['ros2','bag','record','-s','mcap','-o',str(args.bag),*topics(sides)],start_new_session=True)
        try:return child.wait()
        except KeyboardInterrupt:
            child.send_signal(signal.SIGINT)
            return child.wait(timeout=15)
    import yaml
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Int32
    meta=yaml.safe_load((args.bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    source=[f'/tj/hand_{s}2/joint_states' for s in sides]
    available={t['topic_metadata']['name'] for t in meta['topics_with_message_count'] if t['message_count']>0 and t['topic_metadata']['type']=='sensor_msgs/msg/JointState'}
    if not set(source)<=available:p.error('Bag lacks selected hands joint feedback')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=rclpy.create_node('wuji_hand_bag_play')
    seen={};child=None;pub=None
    for s,topic in zip(sides,source):node.create_subscription(JointState,topic,lambda m,s=s:seen.update({s:time.monotonic()}) if len(m.position)==20 else None,qos_profile_sensor_data)
    def interrupt(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupt)
    try:
        deadline=time.monotonic()+5
        while time.monotonic()<deadline and len(seen)<len(sides):rclpy.spin_once(node,timeout_sec=.1)
        if len(seen)<len(sides):raise RuntimeError('Start matching wuji-hand2-driver first')
        if node.count_publishers('/tj/control/footkey2') or any(node.count_publishers(f'/tj/hand_{s}_replay') for s in sides):
            raise RuntimeError('Stop other teleop/pose/replay programs before replay')
        pub=node.create_publisher(Int32,'/tj/control/footkey2',10)
        deadline=time.monotonic()+.5
        while time.monotonic()<deadline:
            pub.publish(Int32(data=4));rclpy.spin_once(node,timeout_sec=.05)
        cmd=['ros2','bag','play',str(args.bag),'--rate',str(args.rate),'--topics',*source,'--remap']
        cmd += [f'{src}:=/tj/hand_{s}_replay' for s,src in zip(sides,source)]
        child=subprocess.Popen(cmd,start_new_session=True)
        while child.poll() is None:
            rclpy.spin_once(node,timeout_sec=.05)
            if any(time.monotonic()-seen.get(s,0)>1 for s in sides):raise RuntimeError('Hand feedback lost; stopping replay')
            pub.publish(Int32(data=4))
        return child.returncode
    except KeyboardInterrupt:return 0
    finally:
        if child is not None and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:child.terminate();child.wait(timeout=3)
        if pub:
            for _ in range(3):pub.publish(Int32(data=0));time.sleep(.03)
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':raise SystemExit(main())
