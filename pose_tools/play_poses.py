#!/usr/bin/env python3
"""Independent pose selection: right 1/2/3, left A/S/D."""
import argparse
import os
from pathlib import Path
import time
from pose_common import (DEFAULT_FILE, JOINT_NAMES, STATE_TOPIC, COMMAND_TOPIC,
                         MODE_TOPIC, Keyboard, blend, load_poses, positions, install_exit_signals)
from pose_common import selected_sides, default_file, paired_serials, load_pose_set, state_topic, command_topic

def main():
    parser = argparse.ArgumentParser(description='左右手独立保持：右手 1/2/3，左手 A/S/D；Q/Ctrl+C 返回保持待机。')
    parser.add_argument('--side', choices=('left', 'right', 'both'), default='right')
    parser.add_argument('--file', type=Path)
    parser.add_argument('--transition', type=float, default=1.0, help='切换过渡秒数，默认 1 秒')
    args = parser.parse_args()
    sides = selected_sides(args.side)
    args.file = args.file or default_file(args.side)
    if not .1 <= args.transition <= 30:
        parser.error('--transition 必须在 0.1 到 30 秒之间')
    try:
        poses = load_pose_set(args.file, sides, paired_serials(sides))
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.error(str(error))
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Int32
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    install_exit_signals()
    node = rclpy.create_node(f'wuji_{args.side}_pose_player')
    latest = {}
    def receive(side, msg):
        try:
            latest[side] = dict(q=positions(msg.name, msg.position), time=time.monotonic())
        except ValueError:
            latest.pop(side, None)
    for side in sides:
        node.create_subscription(JointState, state_topic(side), lambda msg,s=side: receive(s,msg), qos_profile_sensor_data)
    def fresh():
        return all(s in latest and time.monotonic()-latest[s]['time'] < 1 for s in sides)
    active = False
    mode_pub = command_pub = None
    try:
        with Keyboard() as keys:
            print(f'等候 ROS 驱动及关节反馈（{args.side}，最多 10 秒）…', flush=True)
            deadline = time.monotonic() + 10
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
                if keys.read() == 'q':
                    return 0
                if fresh() and all(node.count_subscribers(command_topic(s)) > 0 for s in sides):
                    break
            else:
                raise RuntimeError('未找到所选手的驱动或完整新鲜反馈，未发送动作')
            # Only this player may own mode and user-command publications.
            if node.count_publishers(MODE_TOPIC) or any(node.count_publishers(command_topic(s)) for s in sides):
                raise RuntimeError('存在其他模式/手型发布端；请先关闭手套遥操、其他播放脚本或 Apex 模式驱动')
            mode_pub = node.create_publisher(Int32, MODE_TOPIC, 10)
            command_pubs = {s: node.create_publisher(JointState, command_topic(s), 10) for s in sides}
            indices = {s: 0 for s in sides}
            start = {s: list(latest[s]['q']) for s in sides}
            current = dict(start)
            began = {s: time.monotonic() for s in sides}
            last_check = next_send = time.monotonic()
            key_targets = {'1': ('right', 0), '2': ('right', 1), '3': ('right', 2),
                           'a': ('left', 0), 's': ('left', 1), 'd': ('left', 2)}
            print('当前手型 1/3；右手按 1/2/3，左手按 A/S/D，独立切换。Q/Ctrl+C 退出。', flush=True)
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=.005)
                now = time.monotonic()
                key = keys.read()
                if key == 'q':
                    break
                if not fresh():
                    raise RuntimeError('所选手反馈中断，已停止所有切换指令并返回保持待机')
                if now - last_check >= .5:
                    last_check = now
                    if node.count_publishers(MODE_TOPIC) > 1 or any(node.count_publishers(command_topic(s)) > 1 for s in sides):
                        raise RuntimeError('检测到其他控制发布端，退出姿态播放')
                if key in key_targets:
                    selected, index = key_targets[key]
                    if selected in sides and indices[selected] != index:
                        start[selected] = blend(start[selected], poses[indices[selected]][selected],
                                                now - began[selected], args.transition)
                        indices[selected] = index
                        began[selected] = now
                        label = '右手' if selected == 'right' else '左手'
                        print(f'{label}当前手型 {index + 1}/3', flush=True)
                if now < next_send:
                    continue
                next_send = now + .02
                current = {s: blend(start[s], poses[indices[s]][s], now - began[s], args.transition) for s in sides}
                mode_pub.publish(Int32(data=3))
                stamp = node.get_clock().now().to_msg()
                for side in sides:
                    msg = JointState()
                    msg.header.stamp = stamp
                    msg.name = JOINT_NAMES
                    msg.position = current[side]
                    command_pubs[side].publish(msg)
                active = True
    except (KeyboardInterrupt, EOFError):
        pass
    except (RuntimeError, ValueError) as error:
        print(f'停止：{error}', flush=True)
        return 1
    finally:
        if active and rclpy.ok():
            for _ in range(3):
                mode_pub.publish(Int32(data=0))
                time.sleep(.03)
            print('已返回待机保持；电机仍保持最后姿态。关闭对应驱动可下使能。', flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
