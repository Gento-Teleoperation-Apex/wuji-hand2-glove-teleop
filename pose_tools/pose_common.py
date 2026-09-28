"""Right-hand pose file and terminal keyboard support (no hardware commands)."""
import contextlib
import json
import math
import os
from pathlib import Path
import select
import signal
import sys
import tempfile
import termios
import tty

DEFAULT_FILE = Path.home() / '.local/share/wuji-hand2/right_hand_poses.json'
JOINT_NAMES = [f'j{i}' for i in range(20)]
STATE_TOPIC = '/tj/hand_right2/joint_states'
COMMAND_TOPIC = '/tj/hand_right_user'
MODE_TOPIC = '/tj/control/footkey2'

def selected_sides(side):
    return ['left', 'right'] if side == 'both' else [side]

def default_file(side):
    return DEFAULT_FILE.with_name(f'{side}_hand_poses.json')

def state_topic(side):
    return f'/tj/hand_{side}2/joint_states'

def command_topic(side):
    return f'/tj/hand_{side}_user'

def teach_prefix(side):
    return '/tj/hands2' if side == 'both' else f'/tj/hand_{side}2'

def paired_serials(sides):
    result = {s: os.environ.get(f'WUJI_{s.upper()}_SN') for s in sides}
    if not all(result.values()):
        raise ValueError('请通过安装好的命令启动，以加载左右手配对')
    return result

def load_pose_set(path, sides, serials):
    data = json.loads(Path(path).read_text())
    if data.get('schema_version') == 1 and len(sides) == 1:
        side = sides[0]
        if data.get('side') != side or data.get('hand_serial') != serials[side]:
            raise ValueError('记录文件的手侧或序列号不匹配')
        poses = [{side: p['positions']} for p in data.get('poses', [])]
    elif data.get('schema_version') == 2:
        if data.get('sides') != sides or data.get('hand_serials') != serials:
            raise ValueError('记录文件的左右手或序列号不匹配；单手文件不能当作双手文件')
        poses = [p['positions'] for p in data.get('poses', [])]
    else:
        raise ValueError('文件格式不匹配，需要重新记录所选手的姿态')
    if data.get('units') != 'rad' or len(poses) != 3:
        raise ValueError('需要三组完整手型，角度单位为 rad')
    if any(set(p) != set(sides) for p in poses):
        raise ValueError('某组姿态缺少左手或右手')
    return [{s: positions(data.get('joint_names', []), p[s]) for s in sides} for p in poses]

def install_exit_signals():
    # Keep ROS alive long enough for the player's final standby message.
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, interrupt)

def positions(names, values):
    if len(values) != 20 or len(names) != 20 or set(names) != set(JOINT_NAMES):
        raise ValueError('需要完整的 20 个关节 j0…j19')
    mapping = dict(zip(names, values))
    result = [float(mapping[name]) for name in JOINT_NAMES]
    if not all(math.isfinite(v) for v in result):
        raise ValueError('关节角度包含非有限数值')
    return result

def load_poses(path, serial):
    data = json.loads(Path(path).read_text())
    if data.get('schema_version') != 1 or data.get('side') != 'right' or data.get('units') != 'rad':
        raise ValueError('文件格式、右手标识或角度单位不匹配')
    if not serial or data.get('hand_serial') != serial:
        raise ValueError('记录文件的右手序列号与本机配对不匹配')
    poses = data.get('poses', [])
    if len(poses) != 3:
        raise ValueError('需要先用 X 完整记录三种手型')
    return [positions(data.get('joint_names', []), pose['positions']) for pose in poses]

def atomic_save(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temp = tempfile.mkstemp(prefix=path.name + '.', dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(data, stream, indent=2, ensure_ascii=False, allow_nan=False)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp, path)
    finally:
        if os.path.exists(temp):
            os.unlink(temp)

class Keyboard:
    def __enter__(self):
        if not sys.stdin.isatty():
            raise ValueError('请在交互终端运行，SSH 请使用 ssh -t')
        self.fd = sys.stdin.fileno()
        self.original = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def read(self):
        if select.select([self.fd], [], [], 0)[0]:
            char = os.read(self.fd, 1).decode(errors='ignore').lower()
            if not char:
                raise EOFError('终端已关闭')
            return char
        return None

    def __exit__(self, *args):
        with contextlib.suppress(termios.error):
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.original)

def blend(start, target, elapsed, duration):
    x = min(1.0, max(0.0, elapsed / duration))
    weight = x * x * (3.0 - 2.0 * x)
    return [a + weight * (b - a) for a, b in zip(start, target)]
