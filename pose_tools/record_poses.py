#!/usr/bin/env python3
"""Manual right-hand teaching: B captures/enables, X saves/releases."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import time
from pose_common import DEFAULT_FILE, JOINT_NAMES, STATE_TOPIC, Keyboard, atomic_save, positions, install_exit_signals
from pose_common import selected_sides, default_file, paired_serials, teach_prefix

def main():
    parser = argparse.ArgumentParser(description='手动示教：下使能摆手，B 原位上使能保持，X 保存并下使能，D 放弃当前手型并下使能，Q 退出。')
    parser.add_argument('--side', choices=('left', 'right', 'both'), default='right')
    parser.add_argument('--file', type=Path)
    parser.add_argument('--overwrite', action='store_true', help='重新录制，第一次保存前备份旧文件')
    args = parser.parse_args()
    sides = selected_sides(args.side)
    args.file = args.file or default_file(args.side)
    try:
        serials = paired_serials(sides)
    except ValueError as error:
        parser.error(str(error))
    if args.file.exists() and not args.overwrite:
        parser.error(f'文件已存在：{args.file}；重新录制请加 --overwrite，旧文件会备份')
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from std_srvs.srv import SetBool, Trigger
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    install_exit_signals()
    node = rclpy.create_node(f'wuji_{args.side}_pose_recorder')
    hold_client = node.create_client(SetBool, teach_prefix(args.side) + '/teach_hold')
    finish_client = node.create_client(Trigger, teach_prefix(args.side) + '/teach_finish')
    def call(client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=20)
        if not future.done():
            raise RuntimeError('驱动服务超时，无法确认电机状态；请检查驱动终端')
        response = future.result()
        if not response.success:
            raise RuntimeError(response.message)
        return response.message
    data = dict(schema_version=2, sides=sides, units='rad', hand_serials=serials,
                capture_method='manual_before_enable',
                joint_names=JOINT_NAMES, poses=[])
    backed_up = session = False
    captured = None
    code = 0
    try:
        with Keyboard() as keys:
            print(f'等候新版 ROS 驱动的示教服务（{args.side}）…', flush=True)
            if not hold_client.wait_for_service(timeout_sec=10) or not finish_client.wait_for_service(timeout_sec=2):
                raise RuntimeError(f'找不到示教服务，请启动新版 wuji-hand2-driver --side {args.side}')
            session = True
            print(call(hold_client, SetBool.Request(data=False)), flush=True)
            print(f'记录文件：{args.file}\n手动摆好 → B 保持 → X 保存并释放；重复三次。D 释放重摆，Q 退出。', flush=True)
            last_keys = {}
            while rclpy.ok() and len(data['poses']) < 3:
                rclpy.spin_once(node, timeout_sec=.02)
                key = keys.read()
                now = time.monotonic()
                if key == 'q':
                    break
                if key not in ('b', 'x', 'd'):
                    continue
                quiet = now - last_keys.get(key, 0)
                last_keys[key] = now
                if quiet < .6:
                    continue
                if key == 'b':
                    if captured is not None:
                        print('已经保持当前手型，按 X 保存或 D 释放重摆。', flush=True)
                        continue
                    result = json.loads(call(hold_client, SetBool.Request(data=True)))
                    hands = result.get('hands', {})
                    if set(hands) != set(sides) or any(hands[s].get('hand_serial') != serials[s] for s in sides):
                        raise RuntimeError('驱动反馈的左右手/序列号不匹配')
                    captured = dict(positions={s: positions(JOINT_NAMES, hands[s]['positions']) for s in sides},
                                    captured_at=datetime.now(timezone.utc).isoformat())
                    print('已在手动摆放位置上使能保持。按 X 保存并下使能。', flush=True)
                elif key == 'd':
                    print(call(hold_client, SetBool.Request(data=False)), flush=True)
                    captured = None
                elif captured is None:
                    print('请先手动摆放并按 B 保持，再按 X 保存。', flush=True)
                else:
                    if args.file.exists() and not backed_up:
                        stamp = datetime.now().strftime('%Y%m%d-%H%M%S-%f')
                        backup = args.file.with_name(args.file.name + '.' + stamp + '.bak')
                        shutil.copy2(args.file, backup)
                        print(f'旧文件备份：{backup}', flush=True)
                    backed_up = True
                    slot = len(data['poses']) + 1
                    data['poses'].append(dict(name=f'state_{slot}', **captured))
                    atomic_save(args.file, data)
                    print(f'已保存手型 {slot}/3（B 时上使能前的实际位置）', flush=True)
                    print(call(hold_client, SetBool.Request(data=False)), flush=True)
                    captured = None
            if len(data['poses']) == 3:
                print('三种手型记录完成。', flush=True)
    except (KeyboardInterrupt, EOFError):
        pass
    except (RuntimeError, ValueError, OSError, KeyError) as error:
        print(f'停止：{error}', flush=True)
        code = 1
    finally:
        if session and rclpy.ok():
            try:
                print(call(finish_client, Trigger.Request()), flush=True)
            except Exception as error:
                print(f'未确认下使能：{error}。请在驱动终端 Ctrl+C 停止驱动。', flush=True)
                code = 1
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code

if __name__ == '__main__':
    raise SystemExit(main())
