#!/usr/bin/env python3
"""Small same-origin web control surface for the existing dual ROS pose driver."""
import argparse
from concurrent.futures import Future
import copy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import queue
import secrets
import signal
import sys
import threading
import time
from urllib.parse import urlsplit

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'pose_tools'))
from pose_common import JOINT_NAMES, MODE_TOPIC, blend, command_topic, default_file, load_pose_set, paired_serials, positions, state_topic

SIDES = ('left', 'right')
KEYS = {'1': ('right', 0), '2': ('right', 1), '3': ('right', 2),
        'a': ('left', 0), 's': ('left', 1), 'd': ('left', 2)}

class PoseControl:
    """Pure state machine, separate from ROS/HTTP for deterministic testing."""
    def __init__(self, transition=1.0):
        self.transition = transition
        self.owner = None
        self.heartbeat_at = 0
        self.indices = {s: None for s in SIDES}
        self.starts = self.targets = {}
        self.began = {}
        self.poses = None
        self.message = '等待启用网页控制'

    def start(self, owner, feedback, poses, now):
        if self.owner:
            if self.owner != owner:
                raise ValueError('另一个页面正在控制，请先在原页面暂停')
            self.heartbeat_at = now
            return
        self.poses = copy.deepcopy(poses)
        self.starts = {s: list(feedback[s]) for s in SIDES}
        self.targets = copy.deepcopy(self.starts)
        self.began = {s: now for s in SIDES}
        self.indices = {s: None for s in SIDES}
        self.owner = owner
        self.heartbeat_at = now
        self.message = '正在保持当前位置，请选择手型'

    def authorize(self, owner):
        if not self.owner or owner != self.owner:
            raise ValueError('请先在此页面启用网页控制')

    def sample(self, now):
        return {s: blend(self.starts[s], self.targets[s], now-self.began[s], self.transition) for s in SIDES}

    def select(self, owner, key, now):
        self.authorize(owner)
        if not isinstance(key, str) or key.lower() not in KEYS:
            raise ValueError('仅支持右手 1/2/3、左手 A/S/D')
        side, index = KEYS[key.lower()]
        if self.indices[side] != index:
            self.starts[side] = self.sample(now)[side]
            self.targets[side] = list(self.poses[index][side])
            self.indices[side] = index
            self.began[side] = now
        self.heartbeat_at = now
        self.message = '双手独立保持中'

    def stop(self, message='已暂停切换，驱动保持最后姿态'):
        self.owner = None
        self.message = message

class Runtime:
    def __init__(self, path, transition=1.0, demo=False):
        self.path, self.demo = path, demo
        self.engine = PoseControl(transition)
        self.requests = queue.Queue()
        self.lock = threading.Lock()
        self.snapshot = {'ready': False, 'active': False, 'owner': None, 'message': '正在连接',
                         'sides': {s: {'online': False, 'selected': None} for s in SIDES}, 'demo': demo}
        self.halt = threading.Event()
        self.started = threading.Event()
        self.failure = None
        self.thread = threading.Thread(target=self.run, name='wuji-web-ros', daemon=True)
        self.thread.start()
        if not self.started.wait(12):
            raise RuntimeError('ROS 初始化超时')
        if self.failure:
            raise RuntimeError(self.failure)

    def status(self):
        with self.lock:
            return copy.deepcopy(self.snapshot)

    def action(self, data):
        action = data.get('action')
        if action not in ('start', 'select', 'pause', 'heartbeat'):
            raise ValueError('未知操作')
        client = data.get('client')
        if not isinstance(client, str) or not 8 <= len(client) <= 100:
            raise ValueError('页面会话无效，请刷新')
        future = Future()
        self.requests.put((data, future))
        return future.result(timeout=5)

    def close(self):
        self.halt.set()
        self.thread.join(timeout=5)

    def run(self):
        node = mode_pub = None
        pubs = {}
        latest = {}
        poses = None
        file_error = None
        next_load = next_graph = 0
        conflict = False
        subscribers = False
        rclpy = None
        def drop_control(reason='已暂停切换，驱动保持最后姿态'):
            nonlocal mode_pub, pubs
            if self.engine.owner and mode_pub:
                for _ in range(3):
                    mode_pub.publish(Int32(data=0))
                    time.sleep(.015)
            self.engine.stop(reason)
            if node:
                if mode_pub:
                    node.destroy_publisher(mode_pub)
                for publisher in pubs.values():
                    node.destroy_publisher(publisher)
            mode_pub, pubs = None, {}
        try:
            if self.demo:
                poses = [{s: [(.1*(i+1))*(-1 if s=='left' else 1)]*20 for s in SIDES} for i in range(3)]
            else:
                import rclpy
                from rclpy.signals import SignalHandlerOptions
                from rclpy.qos import qos_profile_sensor_data
                from sensor_msgs.msg import JointState
                from std_msgs.msg import Int32
                rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
                node = rclpy.create_node('wuji_web_pose_control')
                def receive(side, msg):
                    try:
                        latest[side] = (positions(msg.name, msg.position), time.monotonic())
                    except ValueError:
                        latest.pop(side, None)
                for side in SIDES:
                    node.create_subscription(JointState, state_topic(side), lambda m,s=side: receive(s,m), qos_profile_sensor_data)
            self.started.set()
            next_send = 0
            while not self.halt.is_set():
                now = time.monotonic()
                if node:
                    rclpy.spin_once(node, timeout_sec=.002)
                else:
                    for s in SIDES:
                        latest[s] = ([0.]*20, now)
                    subscribers = True
                if not self.engine.owner and now >= next_load and not self.demo:
                    next_load = now + 1
                    try:
                        poses = load_pose_set(self.path, list(SIDES), paired_serials(SIDES))
                        file_error = None
                    except (OSError, ValueError, KeyError, TypeError) as exc:
                        poses = None
                        file_error = ('尚未记录三组双手手型，请先运行 wuji-both-record'
                                      if isinstance(exc, FileNotFoundError) else f'手型文件不可用：{exc}')
                if node and now >= next_graph:
                    next_graph = now + .3
                    own = 1 if mode_pub else 0
                    conflict = (node.count_publishers(MODE_TOPIC) > own or
                                any(node.count_publishers(command_topic(s)) > own for s in SIDES) or
                                any(n.endswith('_pose_recorder') for n in node.get_node_names()))
                    subscribers = all(node.count_subscribers(command_topic(s)) > 0 for s in SIDES)
                online = {s: s in latest and now-latest[s][1] < 1 for s in SIDES}
                reason = (file_error or ('请先停止终端遥操、姿态播放器或录制程序' if conflict else
                          '等待双手驱动，请先运行 wuji-hand2-driver --side both' if not all(online.values()) or not subscribers else ''))
                if self.engine.owner:
                    if conflict:
                        drop_control('检测到其他控制程序，网页已暂停')
                    elif not all(online.values()) or not subscribers:
                        drop_control('双手反馈中断，网页已暂停')
                    elif now-self.engine.heartbeat_at > 3:
                        drop_control('页面连接已断开，网页已暂停')
                try:
                    data, future = self.requests.get_nowait()
                except queue.Empty:
                    data = None
                if data is not None:
                    try:
                        owner, action = data['client'], data['action']
                        if action == 'start':
                            if reason or poses is None:
                                raise ValueError(reason or '手型文件尚未就绪')
                            if not self.engine.owner and node:
                                mode_pub = node.create_publisher(Int32, MODE_TOPIC, 10)
                                pubs = {s: node.create_publisher(JointState, command_topic(s), 10) for s in SIDES}
                            self.engine.start(owner, {s:latest[s][0] for s in SIDES}, poses, now)
                        elif action == 'select':
                            self.engine.select(owner, data.get('key'), now)
                        elif action == 'heartbeat':
                            self.engine.authorize(owner)
                            self.engine.heartbeat_at = now
                        elif action == 'pause':
                            self.engine.authorize(owner)
                            drop_control()
                        future.set_result({'ok': True})
                    except Exception as exc:
                        future.set_exception(ValueError(str(exc)))
                if self.engine.owner and now >= next_send:
                    next_send = now + .02
                    q = self.engine.sample(now)
                    if node:
                        mode_pub.publish(Int32(data=3))
                        stamp = node.get_clock().now().to_msg()
                        for s in SIDES:
                            msg = JointState(name=JOINT_NAMES, position=q[s])
                            msg.header.stamp = stamp
                            pubs[s].publish(msg)
                with self.lock:
                    self.snapshot = dict(ready=not reason and poses is not None, active=bool(self.engine.owner),
                        owner=self.engine.owner, message=self.engine.message if self.engine.owner else reason or self.engine.message,
                        sides={s:dict(online=online[s], selected=self.engine.indices[s]) for s in SIDES},
                        transition=self.engine.transition, file_ready=poses is not None, demo=self.demo)
                time.sleep(.004)
        except Exception as exc:
            self.failure = str(exc)
            with self.lock:
                self.snapshot.update(ready=False, active=False, owner=None, message=f'控制服务异常：{exc}')
            self.started.set()
        finally:
            drop_control('网页服务已关闭')
            if node:
                node.destroy_node()
            if rclpy and rclpy.ok():
                rclpy.shutdown()

class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        if args and str(args[1]) not in ('200',):
            super().log_message(fmt, *args)
    def respond(self, code, body, content='application/json; charset=utf-8'):
        data = body.encode() if isinstance(body, str) else json.dumps(body, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header('Content-Type', content)
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('X-Frame-Options', 'DENY')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)
    def do_GET(self):
        path = urlsplit(self.path).path
        if path == '/':
            html = (Path(__file__).parent/'index.html').read_text()
            self.respond(200, html.replace('__CONTROL_TOKEN__', self.server.control_token), 'text/html; charset=utf-8')
        elif path == '/api/status':
            self.respond(200, self.server.runtime.status())
        else:
            self.respond(404, {'error':'Not found'})
    def do_POST(self):
        if self.path != '/api/action':
            return self.respond(404, {'error':'Not found'})
        if self.headers.get('X-Control-Token') != self.server.control_token:
            return self.respond(403, {'error':'页面已过期，请刷新'})
        origin = self.headers.get('Origin')
        if origin and urlsplit(origin).netloc != self.headers.get('Host'):
            return self.respond(403, {'error':'请求来源不匹配'})
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length < 2048:
                raise ValueError('请求长度无效')
            data = json.loads(self.rfile.read(length))
            if not isinstance(data, dict):
                raise ValueError('请求格式无效')
            self.respond(200, self.server.runtime.action(data))
        except Exception as exc:
            self.respond(409, {'error':str(exc)})

def main():
    p = argparse.ArgumentParser(description='双手姿态网页：右手1/2/3，左手A/S/D。默认只查看，点击启用后才控制。')
    p.add_argument('--host', default='0.0.0.0')
    p.add_argument('--port', type=int, default=8765)
    p.add_argument('--file', type=Path, default=default_file('both'))
    p.add_argument('--transition', type=float, default=1)
    p.add_argument('--demo', action='store_true', help='界面测试，不加载 ROS 或连接设备')
    args = p.parse_args()
    if not math.isfinite(args.transition) or not .1<=args.transition<=30:
        p.error('transition must be 0.1..30')
    runtime = Runtime(args.file, args.transition, args.demo)
    server = ThreadingHTTPServer((args.host,args.port), Handler)
    server.daemon_threads = True
    server.runtime = runtime
    server.control_token = secrets.token_hex(24)
    def stop(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    print(f'Wuji 网页：http://{args.host}:{args.port}（点击启用之前不发布控制指令）',flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        runtime.close()

if __name__=='__main__':
    main()
