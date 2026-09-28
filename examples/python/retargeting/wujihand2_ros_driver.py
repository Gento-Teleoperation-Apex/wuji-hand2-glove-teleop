#!/usr/bin/env python3
"""ROS2 driver bridge for Wuji Hand 2 (Ethernet / wuji_sdk).

Mirrors the gen-1 ``wujihandros2`` topic shape, with a trailing ``2`` on the
hand namespace so both generations can coexist. All Hand2 ROS topics sit
under ``/tj`` (same prefix as ``/tj/control/go_home``):

  Gen-1:  /hand_left/joint_commands   /hand_right/joint_commands
  Hand2:  /tj/hand_left2/joint_commands  /tj/hand_right2/joint_commands

Control mode — ``std_msgs/Int32`` on ``/tj/control/footkey2``:

  0 standby  — freeze; ignore all command sources
  1 teleop   — /tj/hand_*_cmd + /tj/{hand}/joint_commands (glove / teleop)
  2 home     — drive to home_pose.json
  3 user     — /tj/hand_left_user /tj/hand_right_user
  4 replay   — /tj/hand_left_replay /tj/hand_right_replay

Also publishes:
  /tj/{hand_name}/joint_states          sensor_msgs/JointState
  /tj/{hand_name}/tactile               std_msgs/Float32MultiArray
  /tj/{hand_name}/tactile/<finger>      std_msgs/Float32MultiArray

Usage::

    source /opt/ros/humble/setup.bash
    python wujihand2_ros_driver.py --side left --no-footkey
    ros2 topic pub -r 10 /tj/control/footkey2 std_msgs/msg/Int32 "{data: 3}"
"""

from __future__ import annotations

import argparse
import contextlib
import json
import math
import os
import struct
import threading
import time
from typing import Any, Callable, Optional

import numpy as np

from wuji_sdk import DeviceType, JointCommand, SdkManager, WujiHand2

TOTAL_JOINTS = 20
DEFAULT_HAND_NAME = {"left": "hand_left2", "right": "hand_right2"}
# ROS namespace for every Hand2 topic (matches /tj/control/go_home).
TOPIC_NS = "/tj"
# Mode 1 teleop: flat cmd topics (+ namespaced joint_commands for compat)
DEFAULT_TELEOP_CMD = {"left": "hand_left_cmd", "right": "hand_right_cmd"}
# Mode 3 user: dedicated user topics
DEFAULT_USER_CMD = {"left": "hand_left_user", "right": "hand_right_user"}
DEFAULT_REPLAY_CMD = {"left": "hand_left_replay", "right": "hand_right_replay"}
HAND2_KP = 5.0
HAND2_KD = 0.15
HAND2_EFFORT_LIMIT = 1.5
HAND2_QPOS_EMA = 0.35
FOOTKEY_TOPIC = f"{TOPIC_NS}/control/footkey2"
STATE_HZ = 50.0

MODE_STANDBY = 0
MODE_TELEOP = 1
MODE_HOME = 2
MODE_USER = 3
MODE_REPLAY = 4
MODE_NAMES = {
    MODE_STANDBY: "standby",
    MODE_TELEOP: "teleop",
    MODE_HOME: "home",
    MODE_USER: "user",
    MODE_REPLAY: "replay",
}

FINGERS = ("thumb", "index", "middle", "ring", "pinky")
CONTACT_N = 0.2
FIELD_FMT = {
    "i8": "<b",
    "u8": "<B",
    "i16": "<h",
    "u16": "<H",
    "i32": "<i",
    "u32": "<I",
    "f32": "<f",
}
SUMMARY_FIELDS = 6


def nid_to_flat(nid: int) -> Optional[int]:
    bus, node_index = divmod(int(nid) - 1, 5)
    if 0 <= bus < 5 and 0 <= node_index < 4:
        return bus * 4 + node_index
    return None


def parse_side_label(value: object) -> Optional[str]:
    text = str(value).strip().lower()
    if "left" in text:
        return "left"
    if "right" in text:
        return "right"
    return None


def load_home_qpos(path: Optional[str] = None) -> list[float]:
    candidates: list[str] = []
    if path:
        candidates.append(path)
    env = os.environ.get("WUJI_HOME_POSE")
    if env:
        candidates.append(env)
    candidates.append(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "home_pose.json")
    )
    for p in candidates:
        if not p or not os.path.isfile(p):
            continue
        try:
            with open(p, encoding="utf-8") as f:
                data = json.load(f)
            q = data.get("home_qpos")
            if isinstance(q, list) and len(q) >= TOTAL_JOINTS:
                return [float(x) for x in q[:TOTAL_JOINTS]]
        except Exception:
            continue
    return [0.0] * TOTAL_JOINTS


def wait_enabled(hand: WujiHand2, timeout_s: float = 5.0) -> bool:
    deadline = time.monotonic() + timeout_s
    sub = hand.joint_diagnostics().subscribe()
    try:
        while time.monotonic() < deadline:
            time.sleep(0.05)
            frame = sub.recv()
            if frame is None or len(frame.joints) != TOTAL_JOINTS:
                continue
            if all(e.status_word.ext_state == 2 for e in frame.joints):
                return True
    finally:
        sub.close()
    return False


def wait_disabled(hand: WujiHand2, timeout_s: float = 3.0) -> bool:
    deadline = time.monotonic() + timeout_s
    sub = hand.joint_diagnostics().subscribe()
    try:
        while time.monotonic() < deadline:
            frame = sub.recv()
            if frame is not None and len(frame.joints) == TOTAL_JOINTS:
                if all(j.status_word.ext_state == 1 for j in frame.joints):
                    return True
            time.sleep(.02)
    finally:
        sub.close()
    return False


def _make_fingertip_decoder(fmt: dict[str, Any]) -> Callable[[bytes], tuple[list[dict], dict]]:
    pc, stride = fmt["point_count"], fmt["point_stride"]
    expect = pc * stride + fmt["aggregate_stride"]

    def read(defs: list[dict], data: bytes, base: int) -> dict[str, float]:
        out: dict[str, float] = {}
        for d in defs:
            raw = struct.unpack_from(FIELD_FMT[d["type"]], data, base + d["offset"])[0]
            out[d["name"]] = float(raw) * float(d.get("scale", 1.0))
        return out

    def decode(data: bytes) -> tuple[list[dict], dict]:
        if len(data) != expect:
            raise ValueError(f"data length {len(data)} != expected {expect}")
        points = [read(fmt["point_fields"], data, k * stride) for k in range(pc)]
        agg = read(fmt["aggregate_fields"], data, pc * stride)
        return points, agg

    return decode


def _point_force(p: dict[str, float]) -> float:
    return math.sqrt(p.get("fx", 0.0) ** 2 + p.get("fy", 0.0) ** 2 + p.get("fz", 0.0) ** 2)


def _topic(name: Optional[str]) -> Optional[str]:
    if not name:
        return None
    path = name if name.startswith("/") else f"/{name}"
    ns = TOPIC_NS.rstrip("/")
    if path == ns or path.startswith(ns + "/"):
        return path
    return f"{ns}{path}"


class Hand2Slot:
    """One Hand 2 + ROS pubs/subs under /tj/{hand_name}/…"""

    def __init__(
        self,
        node: Any,
        hand: WujiHand2,
        hand_name: str,
        JointState: Any,
        Float32MultiArray: Any,
        MultiArrayDimension: Any,
        qos: Any,
        mode_fn: Callable[[], int],
        *,
        teleop_cmd_topic: Optional[str] = None,
        user_cmd_topic: Optional[str] = None,
        replay_cmd_topic: Optional[str] = None,
        home_qpos: Optional[list[float]] = None,
        enable_tactile: bool = True,
        tactile_calibrate: bool = False,
    ) -> None:
        self.hand = hand
        self.hand_name = hand_name
        self._JointState = JointState
        self._Float32MultiArray = Float32MultiArray
        self._MultiArrayDimension = MultiArrayDimension
        self._mode_fn = mode_fn
        self._home_qpos = list(home_qpos or [0.0] * TOTAL_JOINTS)
        self._pub = hand.joint_command().publish()
        self._cmds = [JointCommand(0.0, 0.0, 0.0) for _ in range(TOTAL_JOINTS)]
        self._q_filt: Optional[np.ndarray] = None
        self._last_cmd: Optional[list[float]] = None
        self._lock = threading.Lock()
        self._motors_on = False
        self._feedback_q = None
        self._feedback_seen = 0.0
        self._tactile_ok = False
        self._decoders: dict[str, Callable] = {}
        self._tactile_subs: dict[str, Any] = {}
        self._tactile_finger_pubs: dict[str, Any] = {}
        self._tactile_summary_pub = None
        self._tactile_frames = 0
        self._tactile_warn_at = 0.0
        self._node_logger = node.get_logger()

        # Connection/standby must not enable motors. Enable only when an
        # accepted command arrives in an explicitly selected active mode.

        state_topic = _topic(f"{hand_name}/joint_states")
        self.state_pub = node.create_publisher(JointState, state_topic, qos)
        # Mode 1: namespaced joint_commands (glove teleop) + flat *_cmd
        ns_cmd = _topic(f"{hand_name}/joint_commands")
        self.cmd_sub = node.create_subscription(
            JointState, ns_cmd, self._on_teleop_cmd, qos
        )
        teleop_flat = _topic(teleop_cmd_topic)
        self.teleop_cmd_sub = None
        if teleop_flat and teleop_flat != ns_cmd:
            self.teleop_cmd_sub = node.create_subscription(
                JointState, teleop_flat, self._on_teleop_cmd, qos
            )

        user_topic = _topic(user_cmd_topic)
        self.user_cmd_sub = None
        if user_topic:
            self.user_cmd_sub = node.create_subscription(
                JointState, user_topic, self._on_user_cmd, qos
            )

        replay_topic = _topic(replay_cmd_topic)
        self.replay_cmd_sub = None
        if replay_topic:
            self.replay_cmd_sub = node.create_subscription(
                JointState, replay_topic, self._on_replay_cmd, qos
            )

        self._state_sub = hand.joint_states().subscribe()
        node.get_logger().info(
            f"Hand2 ROS: SN={hand.serial_number} "
            f"teleop={ns_cmd}+{teleop_flat} user={user_topic} replay={replay_topic} "
            f"state={state_topic}"
        )

        if enable_tactile:
            self._setup_tactile(node, qos, tactile_calibrate)

    def _setup_tactile(self, node: Any, qos: Any, calibrate: bool) -> None:
        if calibrate:
            try:
                self.hand.tactile_calibrate()
                node.get_logger().info(
                    f"[{self.hand_name}] tactile_calibrate() issued (keep fingertips unloaded)"
                )
                time.sleep(0.5)
            except Exception as exc:
                node.get_logger().warn(f"[{self.hand_name}] tactile_calibrate failed: {exc}")

        accessors = {
            "thumb": self.hand.fingertip_thumb_data,
            "index": self.hand.fingertip_index_data,
            "middle": self.hand.fingertip_middle_data,
            "ring": self.hand.fingertip_ring_data,
            "pinky": self.hand.fingertip_pinky_data,
        }
        for i, name in enumerate(FINGERS):
            try:
                info = self.hand.get_fingertip_info(i)
                fmt = json.loads(info.format)
                if fmt.get("v") != 1 or fmt.get("encoding") != "point_array":
                    raise ValueError(f"unsupported format {fmt}")
                self._decoders[name] = _make_fingertip_decoder(fmt)
                self._tactile_subs[name] = accessors[name]().subscribe()
                self._tactile_finger_pubs[name] = node.create_publisher(
                    self._Float32MultiArray,
                    _topic(f"{self.hand_name}/tactile/{name}"),
                    qos,
                )
                rate = getattr(info, "rate_hz", "?")
                node.get_logger().info(
                    f"[{self.hand_name}] tactile/{name}: "
                    f"{fmt['point_count']} pts @ ~{rate} Hz"
                )
            except Exception as exc:
                node.get_logger().warn(
                    f"[{self.hand_name}] no tactile on {name}: {exc}"
                )

        if self._tactile_subs:
            tactile_topic = _topic(f"{self.hand_name}/tactile")
            self._tactile_summary_pub = node.create_publisher(
                self._Float32MultiArray, tactile_topic, qos
            )
            self._tactile_ok = True
            node.get_logger().info(
                f"[{self.hand_name}] tactile summary → {tactile_topic} "
                f"(5×[fx,fy,fz,temp,contacts,max_force])"
            )
        else:
            node.get_logger().warn(f"[{self.hand_name}] tactile disabled (no sensors)")

    def _apply_cmd(self, msg: Any) -> None:
        if len(msg.position) != TOTAL_JOINTS or not all(math.isfinite(float(x)) for x in msg.position):
            return
        q = [float(x) for x in msg.position[:TOTAL_JOINTS]]
        with self._lock:
            if not self._enable_motors():
                return
            self._last_cmd = q
            self._send_locked(q)

    def _enable_motors(self, initial_q=None) -> bool:
        if self._motors_on:
            return True
        self.hand.effort_limit().set(HAND2_EFFORT_LIMIT)
        self.hand.mit_params().set((HAND2_KP, HAND2_KD))
        if initial_q is not None:
            self._q_filt = None
            self._send_locked(initial_q)
        self.hand.enable()
        if initial_q is not None:
            self._send_locked(initial_q)
        if not wait_enabled(self.hand):
            self.hand.disable()
            self._node_logger.error(f'[{self.hand_name}] enable failed; no position command sent')
            return False
        self._motors_on = True
        return True

    def teach_release(self) -> None:
        with self._lock:
            self._last_cmd = None
            self._q_filt = None
            self.hand.disable()
            if not wait_disabled(self.hand):
                raise RuntimeError('未确认全部 20 关节下使能，请检查硬件状态')
            self._motors_on = False

    def capture_teach_pose(self) -> list[float]:
        with self._lock:
            if self._feedback_q is None or time.monotonic() - self._feedback_seen > .5:
                raise RuntimeError('缺少新鲜、完整的 20 关节实际反馈，拒绝上使能')
            if self._motors_on:
                raise RuntimeError('已经处于保持状态；先保存或下使能再摆下一种手型')
            return list(self._feedback_q)

    def teach_hold(self, captured_q=None) -> list[float]:
        q = self.capture_teach_pose() if captured_q is None else list(captured_q)
        if len(q) != TOTAL_JOINTS or not all(math.isfinite(v) for v in q):
            raise RuntimeError('示教姿态必须包含完整的 20 个有限关节角度')
        with self._lock:
            self._last_cmd = None
            try:
                if not self._enable_motors(initial_q=q):
                    raise RuntimeError('上使能失败，未记录手型')
            except Exception:
                self.hand.disable()
                self._motors_on = False
                self._q_filt = None
                raise
            self._last_cmd = q
            return q

    def _on_teleop_cmd(self, msg: Any) -> None:
        if self._mode_fn() != MODE_TELEOP:
            return
        self._apply_cmd(msg)

    def _on_user_cmd(self, msg: Any) -> None:
        if self._mode_fn() != MODE_USER:
            return
        self._apply_cmd(msg)

    def _on_replay_cmd(self, msg: Any) -> None:
        if self._mode_fn() != MODE_REPLAY:
            return
        self._apply_cmd(msg)

    def _send_locked(self, q: list[float]) -> None:
        arr = np.asarray(q, dtype=np.float32)
        if self._q_filt is None:
            self._q_filt = arr.copy()
        else:
            self._q_filt = (1.0 - HAND2_QPOS_EMA) * self._q_filt + HAND2_QPOS_EMA * arr
        for i, p in enumerate(self._q_filt.tolist()):
            self._cmds[i] = JointCommand(float(p), 0.0, 0.0)
        self._pub.send(self._cmds)

    def hold_last(self) -> None:
        with self._lock:
            if self._last_cmd is not None:
                self._send_locked(self._last_cmd)

    def send_home(self) -> None:
        with self._lock:
            if not self._enable_motors():
                return
            self._last_cmd = list(self._home_qpos)
            self._send_locked(self._home_qpos)

    def publish_state(self, stamp: Any) -> None:
        latest = None
        while True:
            frame = self._state_sub.recv()
            if frame is None:
                break
            latest = frame
        if latest is None or not latest.joints:
            return
        pos = [0.0] * TOTAL_JOINTS
        vel = [0.0] * TOTAL_JOINTS
        eff = [0.0] * TOTAL_JOINTS
        seen = set()
        for j in latest.joints:
            idx = nid_to_flat(j.nid)
            if idx is None:
                continue
            seen.add(idx)
            pos[idx] = float(j.position)
            vel[idx] = float(j.velocity)
            eff[idx] = float(j.effort)
        if len(seen) == TOTAL_JOINTS and all(math.isfinite(q) for q in pos):
            self._feedback_q = list(pos)
            self._feedback_seen = time.monotonic()
        else:
            self._feedback_q = None
            return
        msg = self._JointState()
        msg.header.stamp = stamp
        msg.name = [f"j{i}" for i in range(TOTAL_JOINTS)]
        msg.position = pos
        msg.velocity = vel
        msg.effort = eff
        self.state_pub.publish(msg)

    def publish_tactile(self) -> None:
        if not self._tactile_ok:
            return
        summary = [0.0] * (len(FINGERS) * SUMMARY_FIELDS)
        any_frame = False
        for fi, name in enumerate(FINGERS):
            sub = self._tactile_subs.get(name)
            decode = self._decoders.get(name)
            pub = self._tactile_finger_pubs.get(name)
            if sub is None or decode is None or pub is None:
                continue
            latest = None
            while True:
                frame = sub.recv()
                if frame is None:
                    break
                latest = frame
            if latest is None:
                continue
            try:
                points, agg = decode(bytes(latest.data))
            except Exception:
                continue
            any_frame = True
            self._tactile_frames += 1
            forces = [_point_force(p) for p in points]
            contacts = float(sum(1 for f in forces if f > CONTACT_N))
            max_f = float(max(forces) if forces else 0.0)
            base = fi * SUMMARY_FIELDS
            summary[base : base + SUMMARY_FIELDS] = [
                float(agg.get("fx", 0.0)),
                float(agg.get("fy", 0.0)),
                float(agg.get("fz", 0.0)),
                float(agg.get("temperature", 0.0)),
                contacts,
                max_f,
            ]
            flat: list[float] = []
            for p in points:
                flat.extend(
                    [
                        float(p.get("fx", 0.0)),
                        float(p.get("fy", 0.0)),
                        float(p.get("fz", 0.0)),
                    ]
                )
            pt_msg = self._Float32MultiArray()
            pt_msg.layout.dim = [
                self._MultiArrayDimension(
                    label=f"{name}_points", size=len(points), stride=3 * len(points)
                ),
                self._MultiArrayDimension(label="xyz", size=3, stride=3),
            ]
            pt_msg.data = flat
            pub.publish(pt_msg)

        now = time.monotonic()
        if not any_frame:
            if now >= self._tactile_warn_at:
                self._tactile_warn_at = now + 10.0
                self._node_logger.warn(
                    f"[{self.hand_name}] no fingertip data frames "
                    "(check joint_diagnostics.comm.tactile_online_mask; "
                    "0 means tactile slaves offline). Not publishing zeros."
                )
            return

        if self._tactile_summary_pub is not None:
            s_msg = self._Float32MultiArray()
            s_msg.layout.dim = [
                self._MultiArrayDimension(
                    label="fingers",
                    size=len(FINGERS),
                    stride=SUMMARY_FIELDS * len(FINGERS),
                ),
                self._MultiArrayDimension(
                    label="fx_fy_fz_temp_contacts_maxforce",
                    size=SUMMARY_FIELDS,
                    stride=SUMMARY_FIELDS,
                ),
            ]
            s_msg.data = summary
            self._tactile_summary_pub.publish(s_msg)

    def close(self) -> None:
        for sub in self._tactile_subs.values():
            with contextlib.suppress(Exception):
                sub.close()
        with contextlib.suppress(Exception):
            self._state_sub.close()
        with contextlib.suppress(Exception):
            self._pub.close()
        if self._motors_on:
            with contextlib.suppress(Exception):
                self.hand.disable()


class WujiHand2RosDriver:
    def __init__(
        self,
        sides: list[str],
        hand_names: dict[str, str],
        *,
        require_footkey: bool = True,
        sn_filter: Optional[dict[str, str]] = None,
        enable_tactile: bool = True,
        tactile_calibrate: bool = False,
        home_pose_path: Optional[str] = None,
        apex_mode: bool = False,
    ) -> None:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, Float32MultiArray, Int32, MultiArrayDimension
        from std_srvs.srv import SetBool, Trigger

        if not rclpy.ok():
            rclpy.init(args=None)
        self._rclpy = rclpy
        self._node = Node("wujihand2_ros_driver")
        # --no-footkey → start in teleop; otherwise standby until mode published
        self._mode = MODE_TELEOP if not require_footkey else MODE_STANDBY
        self._apex_mode = apex_mode
        self._teaching = False
        self._apex_state = 0
        self._playback_active = False
        self._playback_seen = 0.0
        self._apex_state_topic = os.environ.get('WUJI_APEX_STATE_TOPIC', '/control/switch_state')
        self._playback_key_topic = os.environ.get('WUJI_PLAYBACK_KEY_TOPIC', '/playback_key')
        self._home_available = bool(home_pose_path and os.path.isfile(home_pose_path))
        if apex_mode:
            self._mode = MODE_STANDBY
            self._mode_pub = self._node.create_publisher(Int32, FOOTKEY_TOPIC, 10)
            self._node.create_subscription(Int32, self._apex_state_topic, self._on_apex_state, qos_profile_sensor_data)
            self._node.create_subscription(Bool, self._playback_key_topic, self._on_playback_key,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._slots: dict[str, Hand2Slot] = {}
        self._manager = SdkManager.instance()
        self._home_qpos = load_home_qpos(home_pose_path)

        by_side: dict[str, WujiHand2] = {}
        for d in self._manager.scan():
            if d.device_type != DeviceType.WujiHand2:
                continue
            if sn_filter:
                wanted = {sn_filter[s] for s in sides if s in sn_filter}
                if d.sn not in wanted:
                    continue
            hand = self._manager.connect(sn=d.sn, device_name=f"ros2_{d.sn}")
            if hand.serial_number != d.sn:
                self.close()
                raise RuntimeError(f"Hand identity mismatch: expected {d.sn}, connected {hand.serial_number}")
            side = parse_side_label(hand.handedness().get())
            if side is None:
                self._node.get_logger().warn(f"Skip {d.sn}: unknown handedness")
                continue
            if sn_filter and sn_filter.get(side) and sn_filter[side] != d.sn:
                continue
            by_side[side] = hand

        missing = [side for side in sides if side not in by_side]
        if missing:
            self.close()
            raise SystemExit(f"Required paired hands missing: {missing}; no motors enabled")

        for side in sides:
            hand = by_side.get(side)
            if hand is None:
                self._node.get_logger().error(f"No Wuji Hand 2 for side={side}")
                continue
            name = hand_names[side]
            self._slots[name] = Hand2Slot(
                self._node,
                hand,
                name,
                JointState,
                Float32MultiArray,
                MultiArrayDimension,
                qos_profile_sensor_data,
                mode_fn=lambda: self._mode,
                teleop_cmd_topic=None if apex_mode else DEFAULT_TELEOP_CMD.get(side),
                user_cmd_topic=DEFAULT_USER_CMD.get(side),
                replay_cmd_topic=DEFAULT_REPLAY_CMD.get(side),
                home_qpos=self._home_qpos,
                enable_tactile=enable_tactile,
                tactile_calibrate=tactile_calibrate,
            )

        if not self._slots:
            raise SystemExit("No Hand 2 connected — check Ethernet / IP / power")

        if not apex_mode:
            self._node.create_subscription(Int32, FOOTKEY_TOPIC, self._on_mode, 10)
            self._teach_slots = {side: self._slots[hand_names[side]] for side in sides}
            teach_name = 'hands2' if len(sides) == 2 else hand_names[sides[0]]
            self._node.create_service(SetBool, _topic(f"{teach_name}/teach_hold"), self._on_teach_hold)
            self._node.create_service(Trigger, _topic(f"{teach_name}/teach_finish"), self._on_teach_finish)
        self._node.get_logger().info(
            f"Control mode topic: {FOOTKEY_TOPIC} (std_msgs/Int32) "
            f"0=standby 1=teleop 2=home 3=user 4=replay; "
            f"initial={self._mode} ({MODE_NAMES.get(self._mode, '?')})"
        )

        period = 1.0 / STATE_HZ
        self._node.create_timer(period, self._on_timer)
        self._last_mode = self._mode

    def _release_teach_slots(self):
        failures = []
        for side, slot in self._teach_slots.items():
            try:
                slot.teach_release()
            except Exception as error:
                failures.append(f'{side}: {error}')
        if failures:
            raise RuntimeError('; '.join(failures))

    def _on_teach_hold(self, request, response):
        try:
            if self._node.count_publishers(FOOTKEY_TOPIC):
                raise RuntimeError('请先关闭手套、姿态播放器或其他模式发布端')
            if request.data and not self._teaching:
                raise RuntimeError('请先进入下使能示教，再按 B 保持')
            self._teaching = True
            self._mode = self._last_mode = MODE_STANDBY
            if request.data:
                # Snapshot every selected hand BEFORE enabling either hand.
                snapshots = {side: slot.capture_teach_pose() for side, slot in self._teach_slots.items()}
                try:
                    for side, slot in self._teach_slots.items():
                        slot.teach_hold(snapshots[side])
                except Exception:
                    self._release_teach_slots()
                    raise
                hands = {side: {'positions': snapshots[side], 'hand_serial': slot.hand.serial_number}
                         for side, slot in self._teach_slots.items()}
                result = {'hands': hands}
                if len(hands) == 1:
                    result.update(next(iter(hands.values())))
                response.message = json.dumps(result)
            else:
                self._release_teach_slots()
                response.message = '已确认所选手全部关节下使能，可以手动摆放'
            response.success = True
        except Exception as error:
            response.success = False
            response.message = str(error)
        return response

    def _on_teach_finish(self, request, response):
        try:
            if not self._teaching:
                raise RuntimeError('当前没有手动示教会话')
            self._release_teach_slots()
            self._mode = self._last_mode = MODE_STANDBY
            self._teaching = False
            response.success = True
            response.message = '示教结束，所选手已下使能'
        except Exception as error:
            response.success = False
            response.message = str(error)
        return response

    def _on_apex_state(self, msg: Any) -> None:
        self._apex_state = int(msg.data)
        self._update_apex_mode()

    def _on_playback_key(self, msg: Any) -> None:
        self._playback_active = bool(msg.data)
        self._playback_seen = time.monotonic()
        self._update_apex_mode()

    def _update_apex_mode(self) -> None:
        # Apex: 0 idle, 1 teleop, 2 planner, 3 playback.
        # Never translate Apex planner=2 into Hand2 home=2.
        from std_msgs.msg import Int32
        alive = self._node.count_publishers(self._apex_state_topic) > 0
        replay = self._playback_active and time.monotonic() - self._playback_seen < 1.0
        mode = MODE_STANDBY
        if alive and self._apex_state == 1:
            mode = MODE_TELEOP
        elif alive and self._apex_state == 3 and replay:
            mode = MODE_REPLAY
        self._on_mode(Int32(data=mode))
        self._mode_pub.publish(Int32(data=mode))

    def _on_mode(self, msg: Any) -> None:
        if self._teaching:
            return
        mode = int(msg.data)
        if mode not in MODE_NAMES:
            self._node.get_logger().warn(f"ignore unknown footkey mode={mode}")
            return
        if mode == MODE_HOME and not self._home_available:
            self._node.get_logger().warn('Home rejected: provide a verified --home-pose file first')
            return
        self._mode = mode
        if self._mode != self._last_mode:
            self._last_mode = self._mode
            self._node.get_logger().info(
                f"footkey2 mode = {self._mode} ({MODE_NAMES[self._mode]})"
            )

    def _on_timer(self) -> None:
        if self._apex_mode:
            self._update_apex_mode()
        stamp = self._node.get_clock().now().to_msg()
        mode = self._mode
        for slot in self._slots.values():
            if mode == MODE_STANDBY:
                slot.hold_last()
            elif mode == MODE_HOME:
                slot.send_home()
            # teleop / user / replay: driven by topic callbacks; hold if idle
            elif mode in (MODE_TELEOP, MODE_USER, MODE_REPLAY):
                pass
            slot.publish_state(stamp)
            slot.publish_tactile()

    def spin(self) -> None:
        try:
            self._rclpy.spin(self._node)
        except KeyboardInterrupt:
            pass
        finally:
            self.close()

    def close(self) -> None:
        for slot in self._slots.values():
            slot.close()
        with contextlib.suppress(Exception):
            self._manager.disconnect_all()
        with contextlib.suppress(Exception):
            self._node.destroy_node()
        with contextlib.suppress(Exception):
            if self._rclpy.ok():
                self._rclpy.shutdown()


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Wuji Hand 2 ROS2 driver (Int32 mode footkey2 + tactile)"
    )
    p.add_argument(
        "--side",
        choices=("left", "right", "both"),
        default="both",
        help="Which Hand 2 to drive (default both).",
    )
    p.add_argument(
        "--hand-name",
        default=None,
        help="Override namespace when --side is left or right (default hand_*2).",
    )
    p.add_argument(
        "--no-footkey",
        action="store_true",
        help="Start in mode=1 (teleop). Still listens to /tj/control/footkey2 Int32.",
    )
    p.add_argument(
        "--home-pose",
        default=None,
        help="Path to home_pose.json for mode=2 (default: beside this script).",
    )
    p.add_argument(
        "--no-tactile",
        action="store_true",
        help="Do not publish fingertip tactile topics.",
    )
    p.add_argument(
        "--tactile-calibrate",
        action="store_true",
        help="Call tactile_calibrate() on connect (fingertips must be unloaded).",
    )
    p.add_argument("--apex-mode", action="store_true", help="Follow Apex system mode and playback activity; ignore standalone footkey commands.")
    p.add_argument("--left-sn", default=None, help="Optional left Hand 2 SN filter.")
    p.add_argument("--right-sn", default=None, help="Optional right Hand 2 SN filter.")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    sides = ["left", "right"] if args.side == "both" else [args.side]
    hand_names = dict(DEFAULT_HAND_NAME)
    if args.hand_name:
        if args.side == "both":
            print("Ignoring --hand-name with --side both")
        else:
            hand_names[args.side] = args.hand_name

    sn_filter: Optional[dict[str, str]] = None
    if args.left_sn or args.right_sn:
        sn_filter = {}
        if args.left_sn:
            sn_filter["left"] = args.left_sn
        if args.right_sn:
            sn_filter["right"] = args.right_sn

    driver = WujiHand2RosDriver(
        sides,
        hand_names,
        require_footkey=not args.no_footkey,
        sn_filter=sn_filter,
        enable_tactile=not args.no_tactile,
        tactile_calibrate=args.tactile_calibrate,
        home_pose_path=args.home_pose,
        apex_mode=args.apex_mode,
    )
    driver.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
