#!/usr/bin/env python3
"""
Retargeting example - tuned live teleoperation (keeps official RetargetSession).

Same glove → RetargetSession loop as ``1.teleop_real.py``, with light
application-layer tuning inspired by dex-retargeting / manus_dex_ws practice:

  1. Prefer a **named SDK user** (calibrated glove URDF) instead of forcing the
     default user + builtin URDF.
  2. Per-finger keypoint scaling from the wrist before ``session.step``.
  3. Optional pinky flexion gain / open bias after retarget.
  4. Opposition close: pull thumb tip toward the nearest fingertip when close.
  5. **Control mode / footkey**: Hand2 uses ``std_msgs/Int32`` on
     ``/tj/control/footkey2`` (0 standby / 1 teleop / 2 home / 3 user / 4 replay).
     F7 hold → 1, release → 0. Gen-1 still uses ``Bool`` on ``/control/footkey``.
  6. **Command sink** (``--drive``):
       - ``sdk``: connect Wuji Hand 2 over Ethernet and publish
         ``JointCommand`` directly.
       - ``ros`` + ``wujihand2``: publish to ``/tj/hand_left2|/tj/hand_right2/joint_commands``
         + ``/tj/control/footkey2`` for ``wujihand2_ros_driver.py``.
       - ``ros`` + ``wujihand``: gen-1 ``wujihandros2`` topics (``hand_left`` /
         ``hand_right`` + ``/control/footkey``).
  7. **Footkey + go_home** still work in both modes.

Does **not** replace the SDK retargeter with dex-retargeting — only pre/post
processing around the official API.

Install:
    pip install wuji-sdk numpy pynput
    # ROS2 Humble+ needed for --drive ros / wujihand2_ros_driver

Usage::

    # === Hand 2 via ROS (topics *2) ===
    # terminal A — Hand2 ROS driver
    source /opt/ros/humble/setup.bash
    python wujihand2_ros_driver.py --side both --no-footkey
    # terminal B — glove teleop
    python 2.teleop_tuned.py --drive ros --hand-model wujihand2 --no-footkey

    # === Hand 2 direct SDK (no ROS driver) ===
    python 2.teleop_tuned.py --drive sdk --hand-model wujihand2 --no-footkey

    # === Gen-1 USB via wujihandros2 ===
    # terminal A: ros2 launch wujihand_bringup wujihand.launch.py ...
    python 2.teleop_tuned.py --drive ros --hand-model wujihand --no-footkey
"""

from __future__ import annotations

import argparse
import contextlib
import os
import threading
import time
from pathlib import Path
from typing import Any, Callable, Optional

import numpy as np

from wuji_sdk import (
    DeviceType,
    HandModel,
    Handedness,
    JointCommand,
    RetargetSession,
    SdkManager,
    WujiHand2,
)

from home_pose_service import DEFAULT_CONFIG, HomePoseService

FPS = 120
DEFAULT_HAND_NAME = {"right": "hand_right", "left": "hand_left"}
DEFAULT_HAND_NAME_2 = {"right": "hand_right2", "left": "hand_left2"}
TOPIC_NS = "/tj"
DEFAULT_GO_HOME_SERVICE = f"{TOPIC_NS}/control/go_home"
DEFAULT_HOME_CONFIG = DEFAULT_CONFIG
FOOTKEY_TOPIC_GEN1 = "/control/footkey"
FOOTKEY_TOPIC_HAND2 = f"{TOPIC_NS}/control/footkey2"


def _ns_topic(name: str) -> str:
    path = name if name.startswith("/") else f"/{name}"
    ns = TOPIC_NS.rstrip("/")
    if path == ns or path.startswith(ns + "/"):
        return path
    return f"{ns}{path}"

# Hand 2 direct-drive MIT + EMA (gen-1 filtering lived in wujihandros2)
HAND2_QPOS_EMA = 0.35
HAND2_KP = 5.0
HAND2_KD = 0.15
HAND2_EFFORT_LIMIT = 1.5

# MediaPipe finger base landmark → thumb / index / middle / ring / pinky
_FINGER_BASES = (1, 5, 9, 13, 17)
_TIP_LANDMARKS = (8, 12, 16, 20)  # index / middle / ring / pinky
_TIP_TO_FINGER = {8: 1, 12: 2, 16: 3, 20: 4}
_FLEX_JOINTS = (0, 2, 3)
_THUMB_PINCH_JOINTS = (1, 2, 3)
PINKY_FINGER = 4


class RosJointCommandPublisher:
    """Publish JointState + footkey; optional Trigger go_home service.

    Topics:
      /tj/{hand_name}/joint_commands  sensor_msgs/JointState  (position[20])
      footkey_topic                   Hand2: std_msgs/Int32 mode; gen-1: Bool

    Gen-1 (wujihandros2): hand_left / hand_right + /control/footkey (Bool)
    Hand2 (wujihand2_ros_driver): /tj/hand_*2 + /tj/control/footkey2
      Int32: 0 standby / 1 teleop / 2 home / 3 user / 4 replay
    """

    MODE_STANDBY = 0
    MODE_TELEOP = 1
    MODE_HOME = 2
    MODE_USER = 3
    MODE_REPLAY = 4

    def __init__(
        self,
        hand_names: list[str],
        *,
        footkey_topic: str = FOOTKEY_TOPIC_GEN1,
    ) -> None:
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import JointState
            from std_msgs.msg import Bool, Int32
        except ImportError as exc:
            raise SystemExit(
                "rclpy / sensor_msgs / std_msgs required.\n"
                "  source /opt/ros/humble/setup.bash"
            ) from exc

        if not hand_names:
            raise ValueError("hand_names must not be empty")

        self._rclpy = rclpy
        self._JointState = JointState
        self._Bool = Bool
        self._Int32 = Int32
        self._external_mode = os.environ.get('WUJI_EXTERNAL_MODE') == '1'
        self._external_mode_value = 0
        self._external_mode_seen = 0.0
        self._footkey_topic = footkey_topic
        # Hand2 mode Int32; gen-1 keeps Bool for wujihandros2.
        self._footkey_int = footkey_topic.rstrip("/").endswith("footkey2")
        self._shutdown_rclpy = False
        self._spin_thread: Optional[threading.Thread] = None
        if not rclpy.ok():
            rclpy.init(args=None)
            self._shutdown_rclpy = True

        domain = os.environ.get("ROS_DOMAIN_ID", "0")
        self._node = Node("wuji_teleop_tuned")
        self._cmd_topics: dict[str, str] = {}
        self._pubs = {}
        for name in hand_names:
            cmd_topic = (
                _ns_topic(f"{name}/joint_commands")
                if self._footkey_int
                else f"/{name}/joint_commands"
            )
            self._cmd_topics[name] = cmd_topic
            self._pubs[name] = self._node.create_publisher(
                JointState, cmd_topic, qos_profile_sensor_data
            )
        # Hand2 teleop also publishes flat /tj/hand_*_cmd (mode=1 path).
        self._cmd_pubs: dict[str, Any] = {}
        self._flat_topics: dict[str, str] = {}
        if self._footkey_int and not self._external_mode and os.environ.get('WUJI_CANONICAL_ONLY') != '1':
            for name in hand_names:
                # hand_left2 → /tj/hand_left_cmd
                if name.endswith("2"):
                    flat = _ns_topic(f"{name[:-1]}_cmd")
                    self._flat_topics[name] = flat
                    self._cmd_pubs[name] = self._node.create_publisher(
                        JointState, flat, qos_profile_sensor_data
                    )
        if self._external_mode:
            self._footkey_pub = None
            self._last_footkey = None
            self._node.create_subscription(Int32, footkey_topic, self._on_external_mode, 10)
            fk_type = 'external Apex mode (subscribe only)'
        elif self._footkey_int:
            self._footkey_pub = self._node.create_publisher(Int32, footkey_topic, 10)
            self._last_footkey: Optional[int] = None
            fk_type = "std_msgs/Int32 mode 0..4"
        else:
            self._footkey_pub = self._node.create_publisher(Bool, footkey_topic, 10)
            self._last_footkey = None
            fk_type = "std_msgs/Bool"
        print(f"ROS_DOMAIN_ID={domain}")
        for name in hand_names:
            extra = ""
            if name in self._flat_topics:
                extra = f" + {self._flat_topics[name]}"
            print(
                f"ROS command topic: {self._cmd_topics[name]}{extra} "
                "(sensor_msgs/JointState, position[20])"
            )
        print(f"ROS footkey topic: {footkey_topic} ({fk_type})")

    def start_spin(self) -> None:
        """Background spin so ROS services can be served while teleop loops."""
        if self._spin_thread is not None:
            return
        self._spin_thread = threading.Thread(
            target=self._rclpy.spin,
            args=(self._node,),
            daemon=True,
            name="wuji-teleop-ros-spin",
        )
        self._spin_thread.start()

    def offer_go_home_service(
        self,
        home: HomePoseService,
        *,
        service_name: str = DEFAULT_GO_HOME_SERVICE,
        duration_s: float = 1.5,
    ) -> None:
        """Expose ``std_srvs/Trigger`` for clients (e.g. POST /api/v1/robot/reset)."""
        from std_srvs.srv import Trigger

        def callback(_request, response):
            try:
                if self._footkey_int:
                    # Driver mode=2 streams home_pose.json; also run local go_home
                    # for sdk/ros joint_commands path when mode briefly teleop.
                    self.set_control_mode(self.MODE_HOME)
                    done = home.request_go_home(duration_s)
                    if not done.wait(timeout=max(duration_s, 0.1) + 5.0):
                        response.success = False
                        response.message = "go_home timeout"
                    else:
                        response.success = True
                        response.message = "reset"
                    self.set_control_mode(self.MODE_STANDBY)
                else:
                    self.set_footkey(True)
                    done = home.request_go_home(duration_s)
                    if not done.wait(timeout=max(duration_s, 0.1) + 5.0):
                        response.success = False
                        response.message = "go_home timeout"
                    else:
                        response.success = True
                        response.message = "reset"
            except Exception as exc:
                response.success = False
                response.message = str(exc)
            return response

        self._node.create_service(Trigger, service_name, callback)
        print(
            f"ROS go_home service: {service_name} (std_srvs/Trigger, "
            f"duration={duration_s:.2f}s) → message 'reset'"
        )

    def _on_external_mode(self, msg) -> None:
        self._external_mode_value = int(msg.data)
        self._external_mode_seen = time.monotonic()

    def set_control_mode(self, mode: int) -> None:
        """Publish Hand2 Int32 control mode (0..4). No-op type-wise on gen-1 Bool."""
        if self._external_mode:
            return
        mode = int(mode)
        if self._footkey_int:
            msg = self._Int32()
            msg.data = mode
            self._footkey_pub.publish(msg)
            if self._last_footkey != mode:
                self._last_footkey = mode
                print(f"Published {self._footkey_topic} mode={mode}")
        else:
            self.set_footkey(mode == self.MODE_TELEOP)

    def set_footkey(self, enabled: bool) -> None:
        """Compat: True→mode 1 (teleop), False→mode 0 (standby)."""
        if self._footkey_int:
            self.set_control_mode(
                self.MODE_TELEOP if enabled else self.MODE_STANDBY
            )
            return
        msg = self._Bool()
        msg.data = bool(enabled)
        self._footkey_pub.publish(msg)
        if self._last_footkey is not enabled:
            self._last_footkey = enabled  # type: ignore[assignment]
            print(f"Published {self._footkey_topic} = {enabled}")

    def send(self, qpos: list[float], hand_name: str) -> None:
        if self._external_mode and (self._external_mode_value != self.MODE_TELEOP
                or time.monotonic() - self._external_mode_seen > 1.0):
            return
        pub = self._pubs.get(hand_name)
        if pub is None:
            raise KeyError(f"no publisher for hand {hand_name!r}")
        msg = self._JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.position = [float(x) for x in qpos]
        pub.publish(msg)
        cmd_pub = self._cmd_pubs.get(hand_name)
        if cmd_pub is not None:
            cmd_pub.publish(msg)

    def send_all(self, qpos: list[float]) -> None:
        for name in self._pubs:
            self.send(qpos, name)

    def close(self) -> None:
        with contextlib.suppress(Exception):
            if self._footkey_int:
                self.set_control_mode(self.MODE_STANDBY)
            else:
                self.set_footkey(False)
        with contextlib.suppress(Exception):
            self._node.destroy_node()
        if self._shutdown_rclpy:
            with contextlib.suppress(Exception):
                self._rclpy.shutdown()


def _parse_hand_side(value: object) -> Optional[str]:
    text = str(value).strip().lower()
    if "left" in text:
        return "left"
    if "right" in text:
        return "right"
    return None


def _wait_hand2_enabled(hand: WujiHand2, timeout_s: float = 5.0) -> bool:
    deadline = time.monotonic() + timeout_s
    diag_sub = hand.joint_diagnostics().subscribe()
    try:
        while time.monotonic() < deadline:
            time.sleep(0.05)
            frame = diag_sub.recv()
            if frame is None or not frame.joints:
                continue
            if all(e.status_word.ext_state == 2 for e in frame.joints):
                return True
    finally:
        diag_sub.close()
    return False


class Hand2DirectDriver:
    """Drive Wuji Hand 2 over Ethernet via wuji_sdk (replaces wujihandros2).

    Keep the same send(qpos, hand_name) / set_footkey / send_all surface as
    RosJointCommandPublisher so teleop + HomePoseService stay unchanged.
    Optionally mirrors footkey + go_home onto ROS for Apex.
    """

    def __init__(
        self,
        manager: SdkManager,
        sides: list[str],
        hand_names: dict[str, str],
        *,
        ros_bridge: bool = True,
    ) -> None:
        self._manager = manager
        self._hand_names = hand_names
        self._hands: dict[str, WujiHand2] = {}
        self._pubs: dict[str, Any] = {}
        self._cmds: dict[str, list[JointCommand]] = {}
        self._q_filt: dict[str, Optional[np.ndarray]] = {}
        self._footkey = False
        self._ros: Optional[RosJointCommandPublisher] = None

        self._enabled = set()
        paired = {side: os.environ.get(f"WUJI_{side.upper()}_SN", "") for side in sides}
        by_side = {}
        for d in manager.scan():
            if d.device_type != DeviceType.WujiHand2:
                continue
            if all(paired.values()) and d.sn not in paired.values():
                continue
            hand = manager.connect(sn=d.sn, device_name=f"hand2_{d.sn}")
            side = _parse_hand_side(hand.handedness().get())
            if hand.serial_number != d.sn:
                raise RuntimeError(f"Hand identity mismatch: {d.sn}; check duplicate IPs")
            if side not in sides:
                continue
            if paired[side] and paired[side] != d.sn:
                continue
            if side in by_side:
                raise RuntimeError(f"Multiple {side} hands: configure a paired serial number")
            by_side[side] = hand
        missing = set(sides) - set(by_side)
        if missing:
            raise RuntimeError(f"Required hands missing: {sorted(missing)}; no motors enabled")
        for side in sides:
            hand, name = by_side[side], hand_names[side]
            self._hands[name] = hand
            self._pubs[name] = hand.joint_command().publish()
            self._cmds[name] = [JointCommand(0.0, 0.0, 0.0) for _ in range(20)]
            self._q_filt[name] = None
            print(f"Hand2 direct: {side} SN={hand.serial_number}; waiting for valid gated command")

        if ros_bridge:
            try:
                self._ros = RosJointCommandPublisher(
                    list(self._pubs.keys()),
                    footkey_topic=FOOTKEY_TOPIC_HAND2,
                )
            except SystemExit as exc:
                print(f"ROS bridge skipped ({exc}); footkey/go_home are local-only")
                self._ros = None

    def start_spin(self) -> None:
        if self._ros is not None:
            self._ros.start_spin()

    def offer_go_home_service(
        self,
        home: HomePoseService,
        *,
        service_name: str = DEFAULT_GO_HOME_SERVICE,
        duration_s: float = 1.5,
    ) -> None:
        if self._ros is not None:
            self._ros.offer_go_home_service(
                home, service_name=service_name, duration_s=duration_s
            )
        else:
            print("No ROS: go_home only via teleop HomePoseService / HTTP if enabled")

    def set_footkey(self, enabled: bool) -> None:
        self._footkey = bool(enabled)
        if self._ros is not None:
            self._ros.set_footkey(enabled)

    def send(self, qpos: list[float], hand_name: str) -> None:
        if not self._footkey:
            if hand_name in self._enabled:
                self._pubs[hand_name].send(self._cmds[hand_name])
            return
        pub = self._pubs.get(hand_name)
        if pub is None:
            raise KeyError(f"no Hand2 publisher for {hand_name!r}")
        q = np.asarray(qpos, dtype=np.float32)
        if q.shape != (20,) or not np.isfinite(q).all():
            raise ValueError("Expected 20 finite joint angles")
        if hand_name not in self._enabled:
            hand = self._hands[hand_name]
            sub = hand.joint_states().subscribe()
            current = None
            deadline = time.monotonic() + 3
            try:
                while time.monotonic() < deadline:
                    frame = sub.recv()
                    values = {}
                    if frame is not None:
                        for joint in frame.joints:
                            bus, index = divmod(int(joint.nid)-1, 5)
                            if 0 <= bus < 5 and 0 <= index < 4:
                                values[bus*4+index] = float(joint.position)
                    if len(values) == 20 and all(np.isfinite(v) for v in values.values()):
                        current = np.array([values[i] for i in range(20)])
                        break
                    time.sleep(.01)
            finally:
                sub.close()
            if current is None:
                raise RuntimeError("No complete current joint feedback; refusing enable")
            # Register before enable so cleanup also handles a partial enable failure.
            self._enabled.add(hand_name)
            hand.effort_limit().set(HAND2_EFFORT_LIMIT)
            hand.mit_params().set((HAND2_KP, HAND2_KD))
            pub.send([JointCommand(float(v), 0.0, 0.0) for v in current])
            hand.enable()
            if not _wait_hand2_enabled(hand):
                raise RuntimeError("Hand enable timed out")
            self._q_filt[hand_name] = current
        prev = self._q_filt[hand_name]
        if prev is None:
            filt = q.copy()
        else:
            filt = (1.0 - HAND2_QPOS_EMA) * prev + HAND2_QPOS_EMA * q
        self._q_filt[hand_name] = filt
        cmds = self._cmds[hand_name]
        for i, p in enumerate(filt.tolist()):
            cmds[i] = JointCommand(float(p), 0.0, 0.0)
        pub.send(cmds)
        if self._ros is not None:
            with contextlib.suppress(Exception):
                self._ros.send(qpos, hand_name)

    def send_all(self, qpos: list[float]) -> None:
        for name in self._pubs:
            self.send(qpos, name)

    def close(self) -> None:
        for name, pub in list(self._pubs.items()):
            with contextlib.suppress(Exception):
                pub.close()
            hand = self._hands.get(name)
            if hand is not None and name in self._enabled:
                with contextlib.suppress(Exception):
                    hand.disable()
        if self._ros is not None:
            self._ros.close()


def find_user_by_display_name(manager: SdkManager, display_name: str) -> Optional[dict[str, Any]]:
    matches = [u for u in manager.list_users() if u.get("display_name") == display_name]
    if len(matches) > 1:
        raise RuntimeError(f"Multiple SDK users named {display_name!r}")
    return matches[0] if matches else None


def select_sdk_user(manager: SdkManager, args: argparse.Namespace) -> dict[str, Any]:
    """Pick glove IK user: named / default / first non-default / current."""
    if args.default_user:
        user = manager.switch_to_default_user()
        print("SDK user: Default (builtin URDF)")
        return user

    name = args.user_name
    if name is None:
        named = [u for u in manager.list_users() if not u.get("is_default")]
        if named:
            name = named[0].get("display_name")
            print(f"No --user-name; using first named SDK user {name!r}")
        else:
            user = manager.current_user()
            print(
                "SDK user: current "
                f"{user.get('display_name')!r} (no named user — consider "
                "calibrating under a named user; see wuji_glove/5.calibration.py)"
            )
            return user

    found = find_user_by_display_name(manager, name)
    if found is None:
        raise SystemExit(
            f"SDK user {name!r} not found. Create/switch with "
            "examples/python/wuji_glove/4.user.py, then calibrate."
        )
    user = manager.switch_user(found["user_id"])
    print(f"SDK user: {user.get('display_name')!r} (calibrated URDF if present)")
    if user.get("is_default"):
        print("Warning: default user always uses builtin URDF; calibration is ignored.")
    return user


class FootkeyGate:
    """Hold F7 for teleop mode (same as Apex Teleop); release → standby.

    Hold ``F7`` → mode 1 (teleop) / publish Int32 1 on Hand2 footkey2.
    Release → mode 0 (standby) / publish Int32 0.
    """

    def __init__(self, enabled: bool = True) -> None:
        self._always_on = not enabled
        self._pressed = False
        self._lock = threading.Lock()
        self._listener = None
        self._last_logged: Optional[bool] = None

    @property
    def active(self) -> bool:
        if self._always_on:
            return True
        with self._lock:
            return self._pressed

    def start(self) -> None:
        if self._always_on:
            print("Footkey: disabled (--no-footkey); commands always stream")
            return
        try:
            from pynput import keyboard
        except ImportError as exc:
            raise SystemExit(
                "pynput is required for footkey. Install with: pip install pynput\n"
                "Or run with --no-footkey"
            ) from exc

        def is_f7(key) -> bool:
            return isinstance(key, keyboard.Key) and key == keyboard.Key.f7

        def on_press(key) -> None:
            if not is_f7(key):
                return
            with self._lock:
                self._pressed = True

        def on_release(key) -> None:
            if not is_f7(key):
                return
            with self._lock:
                self._pressed = False

        try:
            self._listener = keyboard.Listener(
                on_press=on_press, on_release=on_release, suppress=False
            )
            self._listener.start()
        except Exception as exc:
            raise SystemExit(
                f"Failed to start footkey keyboard listener: {exc}\n"
                "Need a graphical session (DISPLAY). Or run with --no-footkey"
            ) from exc

        print(
            "Footkey: hold F7 → mode 1 teleop "
            "(release → mode 0 standby; Hand2 /tj/control/footkey2 is Int32)"
        )

    def stop(self) -> None:
        if self._listener is not None:
            with contextlib.suppress(Exception):
                self._listener.stop()
            self._listener = None

    def poll_log(self) -> None:
        if self._always_on:
            return
        on = self.active
        if on != self._last_logged:
            self._last_logged = on
            print(f"Footkey: mode {'1 teleop' if on else '0 standby'}")


def read_keypoints(skeleton_sub) -> Optional[np.ndarray]:
    latest = None
    while True:
        frame = skeleton_sub.recv()
        if frame is None:
            break
        latest = frame
    if latest is None:
        return None
    return np.array([j.pose.position for j in latest.joints], dtype=np.float32)


def scale_finger_keypoints(kp: np.ndarray, finger_scaling: np.ndarray) -> np.ndarray:
    """Scale each finger's landmarks radially from the wrist (MediaPipe index 0)."""
    if np.allclose(finger_scaling, 1.0):
        return kp
    out = kp.copy()
    wrist = kp[0]
    for i, base in enumerate(_FINGER_BASES):
        s = float(finger_scaling[i])
        if abs(s - 1.0) < 1e-6:
            continue
        for k in range(4):
            idx = base + k
            out[idx] = wrist + (kp[idx] - wrist) * s
    return out


def close_opposition_keypoints(
    kp: np.ndarray,
    enable_distance: float,
    close_frac: float,
) -> tuple[np.ndarray, Optional[int], float]:
    """Pull thumb tip and the nearest fingertip toward their midpoint."""
    if close_frac <= 0.0 or enable_distance <= 0.0:
        return kp, None, 0.0

    thumb = kp[4]
    distances = {
        tip: float(np.linalg.norm(kp[tip] - thumb)) for tip in _TIP_LANDMARKS
    }
    closest = min(distances, key=distances.get)
    dist = distances[closest]
    if dist > enable_distance or dist < 1e-9:
        return kp, closest, 0.0

    strength = 1.0 - dist / enable_distance
    alpha = strength * close_frac
    out = kp.copy()
    mid = 0.5 * (thumb + out[closest])
    out[4] = thumb + alpha * (mid - thumb)
    out[closest] = out[closest] + alpha * (mid - out[closest])
    out[closest - 1] = out[closest - 1] + alpha * (mid - out[closest - 1]) * 0.5
    out[3] = out[3] + alpha * (mid - out[3]) * 0.35
    return out, closest, float(strength)


def tune_qpos(
    qpos: np.ndarray,
    pinky_flex_gain: float,
    pinky_open_bias: float,
    pinch_tip: Optional[int],
    pinch_strength: float,
    pinch_flex_boost: float,
) -> np.ndarray:
    """Post-retarget pinky open tune + optional pinch flexion boost."""
    q = np.asarray(qpos, dtype=np.float32)
    need_copy = (
        abs(pinky_flex_gain - 1.0) >= 1e-6
        or abs(pinky_open_bias) >= 1e-9
        or (pinch_flex_boost > 0.0 and pinch_strength > 0.0 and pinch_tip is not None)
    )
    if not need_copy:
        return q
    q = q.copy()

    base = PINKY_FINGER * 4
    for j in _FLEX_JOINTS:
        q[base + j] = q[base + j] * pinky_flex_gain - pinky_open_bias

    if pinch_flex_boost > 0.0 and pinch_strength > 0.0 and pinch_tip is not None:
        add = pinch_flex_boost * pinch_strength
        for j in _THUMB_PINCH_JOINTS:
            q[j] = q[j] + add
        finger = _TIP_TO_FINGER[pinch_tip]
        fbase = finger * 4
        for j in _FLEX_JOINTS:
            q[fbase + j] = q[fbase + j] + add

    return q


def parse_side_label(value: object) -> Optional[str]:
    """Map glove.hand_side() to 'left' or 'right'."""
    text = str(value).strip().lower()
    if "left" in text:
        return "left"
    if "right" in text:
        return "right"
    return None


def teleop(
    arms: list[tuple[Any, Handedness, str, Callable[[list[float]], None]]],
    model: HandModel,
    set_footkey: Callable[[bool], None],
    finger_scaling: np.ndarray,
    pinky_flex_gain: float,
    pinky_open_bias: float,
    opposition_enable_distance: float,
    opposition_close: float,
    pinch_flex_boost: float,
    footkey: FootkeyGate,
    home: Optional[HomePoseService],
) -> None:
    states: list[dict[str, Any]] = []
    for glove, side, hand_name, send in arms:
        session = RetargetSession.for_hand(model, side=side)
        side_label = "left" if side == Handedness.Left else "right"
        print(
            f"Retarget side={side_label} glove={glove.serial_number} "
            f"→ {hand_name}  Tuning: finger_scaling="
            f"{finger_scaling.tolist()} pinky_flex_gain={pinky_flex_gain} "
            f"pinky_open_bias={pinky_open_bias} "
            f"opposition_enable_distance={opposition_enable_distance} "
            f"opposition_close={opposition_close} "
            f"pinch_flex_boost={pinch_flex_boost}"
        )
        states.append(
            {
                "sub": glove.hand_skeleton().subscribe(),
                "session": session,
                "send": send,
                "last_kp": None,
                "seen": time.monotonic(),
                "last_sent": None,
            }
        )
    if home is not None:
        print(f"Home loaded from {home.config_path}; ROS go_home service ready")
    print(f"Teleoperating {len(states)} hand(s) (Ctrl+C to stop)...")

    budget = 1.0 / FPS
    while True:
        frame_start = time.monotonic()
        footkey.poll_log()

        # Service-triggered go_home takes the command channel until done.
        if home is not None and home.has_pending:
            set_footkey(True)
            home.poll()
            continue

        enabled = footkey.active
        set_footkey(enabled)

        for st in states:
            kp = read_keypoints(st["sub"])
            if kp is None:
                if time.monotonic() - st["seen"] > 1.:
                    raise RuntimeError("Glove skeleton missing for 1 second; stopping control")
                kp = st["last_kp"]
                if kp is None:
                    continue
            else:
                if kp.shape != (21, 3) or not np.isfinite(kp).all():
                    raise ValueError("Invalid glove skeleton")
                st["seen"] = time.monotonic()
                st["last_kp"] = kp

            if not enabled:
                if st["last_sent"] is not None:
                    st["send"](st["last_sent"])
                continue

            kp = scale_finger_keypoints(kp, finger_scaling)
            kp, pinch_tip, pinch_strength = close_opposition_keypoints(
                kp, opposition_enable_distance, opposition_close
            )
            qpos = tune_qpos(
                st["session"].step(kp),
                pinky_flex_gain,
                pinky_open_bias,
                pinch_tip,
                pinch_strength,
                pinch_flex_boost,
            )
            cmd = qpos.tolist()
            st["send"](cmd)
            st["last_sent"] = cmd

        dt = time.monotonic() - frame_start
        if dt < budget:
            time.sleep(budget - dt)


def run_teleop(
    manager: SdkManager,
    sides: list[str],
    hand_names: dict[str, str],
    hand_model: HandModel,
    drive: str,
    finger_scaling: np.ndarray,
    pinky_flex_gain: float,
    pinky_open_bias: float,
    opposition_enable_distance: float,
    opposition_close: float,
    pinch_flex_boost: float,
    use_footkey: bool,
    home: Optional[HomePoseService],
    go_home_service: str,
    go_home_duration: float,
) -> int:
    gloves_by_side: dict[str, Any] = {}
    paired = {s: os.environ.get(f"WUJI_{s.upper()}_GLOVE_SN", "") for s in sides}
    paired_sns = {sn for sn in paired.values() if sn}
    for d in manager.scan():
        print(f"  SN={d.sn}, Type={d.device_type}, Address={d.address}")
        if d.device_type != DeviceType.WujiGlove:
            continue
        if paired_sns and d.sn not in paired_sns:
            continue
        glove = manager.connect(sn=d.sn, device_name=f"glove_{d.sn}")
        if glove.serial_number != d.sn:
            manager.disconnect_all()
            raise RuntimeError(f"Glove identity mismatch: expected {d.sn}, connected {glove.serial_number}; check duplicate IPs")
        label = parse_side_label(glove.hand_side().get())
        expected = next((s for s, sn in paired.items() if sn == d.sn), None)
        if expected is not None and label != expected:
            manager.disconnect_all()
            raise RuntimeError(f"Paired {expected} glove {d.sn} reports side={label}")
        if label is None:
            print(f"Skip glove {d.sn}: unknown hand_side")
            glove.disconnect()
            continue
        if label not in sides:
            glove.disconnect()
            continue
        if label in gloves_by_side:
            raise RuntimeError(f"Multiple {label} gloves: configure paired serial numbers")
        gloves_by_side[label] = glove

    missing = [s for s in sides if s not in gloves_by_side]
    if missing:
        print(f"Paired gloves unavailable: {', '.join(missing)}")
        manager.disconnect_all()
        return 1

    ordered = [s for s in ("left", "right") if s in gloves_by_side]
    for s in sides:
        if s not in gloves_by_side:
            print(f"Skip {s}: no matching glove")

    if not ordered:
        print("No matching Wuji Glove found")
        manager.disconnect_all()
        return 1

    if drive == "sdk":
        sink: Any = Hand2DirectDriver(
            manager, ordered, hand_names, ros_bridge=False
        )
        print("Drive: sdk (Wuji Hand 2 direct — no ROS driver node)")
    else:
        footkey_topic = (
            FOOTKEY_TOPIC_HAND2
            if hand_model == HandModel.WujiHand2
            else FOOTKEY_TOPIC_GEN1
        )
        sink = RosJointCommandPublisher(
            [hand_names[s] for s in ordered],
            footkey_topic=footkey_topic,
        )
        if hand_model == HandModel.WujiHand2:
            print(
                "Drive: ros → wujihand2_ros_driver "
                f"({', '.join(_ns_topic(hand_names[s]+'/joint_commands') for s in ordered)})"
            )
        else:
            print("Drive: ros → wujihandros2 (gen-1 USB)")

    def send_all_with_footkey(q: list[float]) -> None:
        sink.set_footkey(True)
        sink.send_all(q)

    if home is not None:
        home.bind_sender(send_all_with_footkey)
        sink.offer_go_home_service(
            home, service_name=go_home_service, duration_s=go_home_duration
        )
    sink.start_spin()

    footkey = FootkeyGate(enabled=use_footkey)

    arms = [
        (
            gloves_by_side[s],
            Handedness.Left if s == "left" else Handedness.Right,
            hand_names[s],
            lambda q, name=hand_names[s]: sink.send(q, name),
        )
        for s in ordered
    ]

    try:
        footkey.start()
        teleop(
            arms,
            hand_model,
            sink.set_footkey,
            finger_scaling,
            pinky_flex_gain,
            pinky_open_bias,
            opposition_enable_distance,
            opposition_close,
            pinch_flex_boost,
            footkey,
            home,
        )
    except KeyboardInterrupt:
        pass
    finally:
        footkey.stop()
        sink.close()
        manager.disconnect_all()

    return 0


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=(
            "Tuned Wuji glove→hand teleop. "
            "--drive sdk: Hand2 Ethernet direct; "
            "--drive ros: gen-1 wujihandros2 topics."
        )
    )
    p.add_argument("--profile", choices=("official", "tuned"), default="tuned",
                   help="official: SDK RetargetSession baseline, no application finger/pinch tuning")
    user = p.add_mutually_exclusive_group()
    user.add_argument(
        "--user-name",
        default=None,
        help="Named SDK user for glove IK (default: first non-default user).",
    )
    user.add_argument(
        "--default-user",
        action="store_true",
        help="Force default SDK user + builtin URDF (same as 1.teleop_real.py).",
    )
    p.add_argument(
        "--side",
        choices=("left", "right", "both"),
        default="both",
        help="Retarget handedness (default: both left+right gloves).",
    )
    p.add_argument(
        "--hand-name",
        default=None,
        help="ROS namespace when --side is left or right (default: hand_left / hand_right).",
    )
    p.add_argument(
        "--hand-model",
        choices=("wujihand", "wujihand2"),
        default="wujihand2",
        help="Retarget HandModel (default: wujihand2).",
    )
    p.add_argument(
        "--drive",
        choices=("sdk", "ros"),
        default=None,
        help=(
            "Command sink: sdk=direct Hand2 Ethernet; "
            "ros=ROS topics (Hand2→/tj/hand_*2 + /tj/control/footkey2 via wujihand2_ros_driver; "
            "gen-1→hand_* + footkey via wujihandros2). "
            "Default: sdk if --hand-model wujihand2 else ros."
        ),
    )
    p.add_argument(
        "--finger-scaling",
        type=float,
        nargs=5,
        metavar=("THUMB", "INDEX", "MIDDLE", "RING", "PINKY"),
        default=None,
        help="Per-finger keypoint scale from wrist (default: 1 1 1 1 1.1).",
    )
    p.add_argument(
        "--pinky-scale",
        type=float,
        default=None,
        help="Override pinky entry of finger-scaling (default 1.1).",
    )
    p.add_argument(
        "--pinky-flex-gain",
        type=float,
        default=0.9,
        help="Multiply pinky flexion joints after retarget (default 0.9).",
    )
    p.add_argument(
        "--pinky-open-bias",
        type=float,
        default=0.05,
        help="Subtract from pinky flexion joints after gain, radians (default 0.05).",
    )
    p.add_argument(
        "--opposition-enable-distance",
        type=float,
        default=0.10,
        help="Tip distance (m) below which opposition close starts (default 0.10).",
    )
    p.add_argument(
        "--opposition-close",
        type=float,
        default=0.55,
        help="Fraction of thumb↔tip gap to close in keypoint space (default 0.55).",
    )
    p.add_argument(
        "--pinch-flex-boost",
        type=float,
        default=0.12,
        help="Extra flexion (rad) on thumb + closest finger when pinching (default 0.12).",
    )
    p.add_argument(
        "--no-footkey",
        action="store_true",
        help="Disable footkey gate (always stream). Default requires hold F7 (Apex Teleop).",
    )
    p.add_argument(
        "--home-config",
        type=Path,
        default=DEFAULT_HOME_CONFIG,
        help=f"Home pose JSON (default: {DEFAULT_HOME_CONFIG.name}).",
    )
    p.add_argument(
        "--no-home",
        action="store_true",
        help="Do not load home / do not advertise go_home service.",
    )
    p.add_argument(
        "--go-home-service",
        default=DEFAULT_GO_HOME_SERVICE,
        help=f"ROS Trigger service name (default: {DEFAULT_GO_HOME_SERVICE}).",
    )
    p.add_argument(
        "--go-home-duration",
        type=float,
        default=1.5,
        help="Seconds to stream home pose when service is called (default 1.5).",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()
    if args.profile == "official":
        args.default_user = True
        args.user_name = None
        args.finger_scaling = [1.] * 5
        args.pinky_scale = 1.
        args.pinky_flex_gain = 1.
        args.pinky_open_bias = 0.
        args.opposition_close = 0.
        args.pinch_flex_boost = 0.
    sides = ["left", "right"] if args.side == "both" else [args.side]
    hand_model = HandModel.WujiHand if args.hand_model == "wujihand" else HandModel.WujiHand2
    drive = args.drive
    if drive is None:
        drive = "sdk" if args.hand_model == "wujihand2" else "ros"
    if drive == "sdk" and args.hand_model != "wujihand2":
        print("Note: --drive sdk is for Hand 2; forcing --hand-model wujihand2")
        hand_model = HandModel.WujiHand2

    # Default ROS namespaces: gen-1 hand_* ; Hand2 hand_*2
    if hand_model == HandModel.WujiHand2 and drive == "ros":
        hand_names = dict(DEFAULT_HAND_NAME_2)
    else:
        hand_names = dict(DEFAULT_HAND_NAME)
    if args.hand_name:
        if args.side == "both":
            print("Ignoring --hand-name because --side both uses left/right defaults")
        else:
            hand_names[args.side] = args.hand_name

    finger_scaling = np.array(
        args.finger_scaling if args.finger_scaling is not None else [1.0, 1.0, 1.0, 1.0, 1.1],
        dtype=np.float32,
    )
    if args.pinky_scale is not None:
        finger_scaling[PINKY_FINGER] = args.pinky_scale

    home: Optional[HomePoseService] = None
    if not args.no_home:
        home = HomePoseService.try_from_config(args.home_config)
        if home is None:
            print(f"No home file at {args.home_config} — run: python 3.save_home.py")
            print("ROS go_home service will NOT be advertised.")
        else:
            print(f"Loaded home from {args.home_config.resolve()}")

    manager = SdkManager.instance()
    previous_user = manager.current_user()
    exit_code = 0
    try:
        select_sdk_user(manager, args)
        exit_code = run_teleop(
            manager,
            sides,
            hand_names,
            hand_model,
            drive,
            finger_scaling,
            args.pinky_flex_gain,
            args.pinky_open_bias,
            args.opposition_enable_distance,
            args.opposition_close,
            args.pinch_flex_boost,
            use_footkey=not args.no_footkey,
            home=home,
            go_home_service=args.go_home_service,
            go_home_duration=args.go_home_duration,
        )
    finally:
        manager.disconnect_all()
        try:
            manager.switch_user(previous_user["user_id"])
        except Exception as exc:
            print(f"Failed to restore previous SDK user: {exc}")
            exit_code = 1

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
