#!/usr/bin/env python3
"""
Retargeting example - live teleoperation from a Wuji Glove.

The SDK exposes only the pure retarget interface (``RetargetSession``). Teleop —
read keypoints → retarget → drive the hand — is plain example code you can adapt,
shown end to end below: read the glove's hand keypoints, retarget each frame, and
drive a connected Wuji Hand or Wuji Hand 2.

To drive from a different keypoint source (camera / MediaPipe / VR), replace the
glove read with your own: ``RetargetSession.step`` accepts any ``(21, 3)`` float32
array of MediaPipe-format landmarks and returns a 20-value joint command in
firmware order, so the result is sent to the hand as-is.

The session is built with ``RetargetSession.for_hand(model, side=SIDE)`` — the hand
model selects the builtin tuning config internally, so there is no config path to
manage. Configuring and enabling the hand's motors is the caller's responsibility,
shown here in configure_wuji_hand_2 / configure_wuji_hand.

This example pairs each glove with the matching-handedness Wuji Hand and
teleoperates every pair at once (left glove → left hand, right glove → right
hand). A single pair still works if only one side is connected.

This example runs the **glove** on the SDK built-in default hand URDF: it
switches to the default SDK user before connecting — the default user runs the
glove on the built-in default hand URDF — and restores the previously selected
user on exit. See ``use_builtin_urdf_user``. Use a named SDK user to run with a
calibrated hand model: create one, switch to it, then calibrate the glove under
it — calibrating under the default user has no effect, since the default user
always runs on the built-in URDF.

Install the retargeting runtime dependencies first (numpy for keypoint/qpos arrays):

    pip install wuji-sdk numpy

Usage:
    python 1.teleop_real.py

Application 2.3.0 additions: generation/side selection, validated motor options,
duplicate-side rejection and stale-glove timeout. See --help. No footkey gate.
"""

import argparse
import contextlib
import math
import threading
import time

import numpy as np

from wuji_sdk import (
    DeviceType,
    HandModel,
    Handedness,
    JointCommand,
    LowPass,
    RetargetSession,
    SdkManager,
    WujiHand2,
)

FPS = 120  # target loop rate (ceiling); a slow frame lowers the rate, never bursts.
HAND1_EFFORT_LIMIT = 1.5
HAND1_LOWPASS_HZ = 5.0
STALE_TIMEOUT = 1.0
# Hand 2 has no realtime LowPass like gen-1; EMA on qpos reduces command chatter.
HAND2_QPOS_EMA = 0.35
HAND2_KP = 5.0
HAND2_KD = 0.15
HAND2_EFFORT_LIMIT = 1.5  # Amps


def wait_hand2_enabled(hand, timeout_s: float = 5.0) -> bool:
    """Block until all online joints report Enabled (ext_state==2)."""
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


def configure_wuji_hand_2(hand):
    """Configure + enable a Wuji Hand 2; return the retarget HandModel."""
    # MIT impedance: hold each joint at its commanded position. The
    # firmware defaults to MIT control mode (control mode is not set from Python).
    with contextlib.suppress(Exception):
        hand.clear_fault()
    hand.effort_limit().set(HAND2_EFFORT_LIMIT)
    hand.mit_params().set((HAND2_KP, HAND2_KD))  # (kp, kd), broadcast to all joints
    hand.enable()
    if not wait_hand2_enabled(hand):
        raise RuntimeError(f"[{hand.serial_number}] enable timeout")
    return HandModel.WujiHand2


def configure_wuji_hand(hand):
    """Configure + enable a first-gen Wuji Hand; return the retarget HandModel."""
    hand.set_all_effort_limit(HAND1_EFFORT_LIMIT)  # Amps
    hand.enable()
    return HandModel.WujiHand


def use_builtin_urdf_user(manager):
    """Switch to the default SDK user so the glove uses the built-in URDF.

    The default SDK user runs the glove on the built-in default hand URDF rather
    than a per-user IK calibration. Returns the previously selected user so
    main() can restore it on exit.
    """
    previous = manager.current_user()
    manager.switch_to_default_user()
    return previous


def read_keypoints(skeleton_sub):
    """Read the glove's latest hand_skeleton frame as a (21, 3) float32 array.

    The glove publishes hand_skeleton faster than this loop consumes it, and
    recv() returns the OLDEST unread frame — taking one per loop falls behind.
    Drain every queued frame, keep only the latest. Returns None if no frame is
    available this tick.
    """
    latest = None
    while True:
        frame = skeleton_sub.recv()
        if frame is None:
            break
        latest = frame
    if latest is None:
        return None
    return np.array([j.pose.position for j in latest.joints], dtype=np.float32)


def parse_side(value) -> str | None:
    """Map glove/hand side labels to 'left' or 'right'."""
    text = str(value).strip().lower()
    if "left" in text:
        return "left"
    if "right" in text:
        return "right"
    return None


def hand_side(hand) -> str | None:
    """Wuji Hand 2: handedness().get(); gen-1 Hand: handedness_name()."""
    if isinstance(hand, WujiHand2):
        return parse_side(hand.handedness().get())
    return parse_side(hand.handedness_name())


def side_enum(side: str) -> Handedness:
    return Handedness.Left if side == "left" else Handedness.Right


def teleop(glove, model, send, side, stop, smooth_qpos: bool = False):
    """Build the retargeting session + glove source, then loop read → retarget → send.

    Called *after* the hand's command channel is already open (see run_pair):
    open the command channel before session initialization so device
    communication remains active during setup.
    """
    session = RetargetSession.for_hand(model, side=side)
    skeleton_sub = glove.hand_skeleton().subscribe()

    budget = 1.0 / FPS
    last_kp = None
    q_filt = None
    n = 0
    t0 = time.monotonic()
    last_frame_time = t0
    try:
        while not stop.is_set():
            frame_start = time.monotonic()
            kp = read_keypoints(skeleton_sub)
            if kp is None:
                if time.monotonic() - last_frame_time > STALE_TIMEOUT:
                    raise RuntimeError(f"[{side}] no fresh glove skeleton for {STALE_TIMEOUT}s")
                kp = last_kp  # no fresh frame: hold the last pose
                if kp is None:
                    time.sleep(budget)  # nothing yet — wait out the budget
                    continue
            else:
                if kp.shape != (21, 3) or not np.isfinite(kp).all():
                    raise ValueError('Invalid glove skeleton')
                last_kp = kp
                last_frame_time = time.monotonic()
            qpos = np.asarray(session.step(kp))  # (20,) firmware order
            if qpos.shape != (20,) or not np.isfinite(qpos).all():
                raise ValueError('Invalid retarget output')
            if smooth_qpos:
                if q_filt is None:
                    q_filt = qpos.copy()
                else:
                    q_filt = (1.0 - HAND2_QPOS_EMA) * q_filt + HAND2_QPOS_EMA * qpos
                send(q_filt.tolist())
            else:
                send(qpos.tolist())
            n += 1
            if n % 240 == 0:
                hz = n / max(time.monotonic() - t0, 1e-6)
                print(f"[{side}] loop ~{hz:.0f} Hz", flush=True)
            dt = time.monotonic() - frame_start
            if dt < budget:
                time.sleep(budget - dt)
    finally:
        skeleton_sub.close()


def run_pair(glove, hand, side, stop):
    """Enable one hand and stream glove keypoints into it until ``stop`` is set."""
    is_hand2 = isinstance(hand, WujiHand2)
    print(f"[{side}] glove={glove.serial_number} hand={hand.serial_number}")

    # Open the hand's command channel first — before teleop() builds the
    # retargeting session — so device communication remains active during setup.
    try:
        model = configure_wuji_hand_2(hand) if is_hand2 else configure_wuji_hand(hand)
        if is_hand2:
            publisher = hand.joint_command().publish()
            cmds = [JointCommand(0.0, 0.0, 0.0) for _ in range(20)]

            def send_hand2(q):
                for i, p in enumerate(q):
                    cmds[i] = JointCommand(p, 0.0, 0.0)
                publisher.send(cmds)

            try:
                teleop(
                    glove,
                    model,
                    send_hand2,
                    side_enum(side),
                    stop,
                    smooth_qpos=True,
                )
            finally:
                publisher.close()
        else:
            with hand.realtime_controller(LowPass(cutoff_hz=HAND1_LOWPASS_HZ)) as controller:
                teleop(
                    glove,
                    model,
                    controller.set_target_position,
                    side_enum(side),
                    stop,
                )
    finally:
        with contextlib.suppress(Exception):
            hand.disable()


def run_teleop(manager, hand_model="auto", requested_side="auto") -> int:
    """Connect matching glove+hand pairs and teleoperate until interrupted."""

    glove_devs = []
    hand_devs = []
    for d in manager.scan():
        print(f"  SN={d.sn}, Type={d.device_type}, Address={d.address}")
        allowed = ((DeviceType.WujiHand, DeviceType.WujiHand2) if hand_model == "auto" else
                   (DeviceType.WujiHand,) if hand_model == "wujihand" else (DeviceType.WujiHand2,))
        if d.device_type in allowed:
            hand_devs.append(d)
        elif d.device_type == DeviceType.WujiGlove:
            glove_devs.append(d)

    if not glove_devs:
        print("No Wuji Glove found")
        return 1
    if not hand_devs:
        print("No Wuji Hand / Wuji Hand 2 found")
        return 1

    gloves_by_side: dict[str, object] = {}
    for d in glove_devs:
        glove = manager.connect(sn=d.sn, device_name=f"glove_{d.sn}")
        if glove.serial_number != d.sn:
            raise RuntimeError(f"Glove SN mismatch at {d.address}; check duplicate IPs")
        side = parse_side(glove.hand_side().get())
        if side is None:
            print(f"Skip glove {d.sn}: unknown hand_side")
            continue
        if requested_side in ("left", "right") and side != requested_side:
            continue
        if side in gloves_by_side:
            raise RuntimeError(f"Multiple {side} gloves; connect only one per side")
        gloves_by_side[side] = glove

    hands_by_side: dict[str, object] = {}
    for d in hand_devs:
        hand = manager.connect(sn=d.sn, device_name=f"hand_{d.sn}")
        # Gen-1 WujiHand exposes its product/model identifier as
        # `serial_number`; the USB discovery SN is available only on the
        # discovered-device record. Connecting by d.sn already selects the
        # requested unit, so comparing these values rejects valid gen-1 hands.
        # Keep the identity check for gen-2, whose serial_number is the device SN.
        if d.device_type == DeviceType.WujiHand2 and hand.serial_number != d.sn:
            raise RuntimeError(f"Hand SN mismatch at {d.address}")
        side = hand_side(hand)
        if side is None:
            print(f"Skip hand {d.sn}: unknown handedness")
            continue
        if requested_side in ("left", "right") and side != requested_side:
            continue
        if side in hands_by_side:
            raise RuntimeError(f"Multiple {side} hands; select --hand-model or disconnect extras")
        hands_by_side[side] = hand

    pairs = []
    for side in ("left", "right"):
        if requested_side in ("left", "right") and side != requested_side:
            continue
        glove = gloves_by_side.get(side)
        hand = hands_by_side.get(side)
        if glove is not None and hand is not None:
            pairs.append((glove, hand, side))
        elif requested_side != "auto":
            raise RuntimeError(f"Missing requested {side} glove/hand pair")
        elif glove is not None or hand is not None:
            missing = "hand" if glove is not None else "glove"
            print(f"Skip {side}: no matching {missing}")

    if not pairs:
        print("No matching glove+hand pairs (left/right)")
        manager.disconnect_all()
        return 1

    stop = threading.Event()
    errors: list[BaseException] = []

    def worker(glove, hand, side):
        try:
            run_pair(glove, hand, side, stop)
        except Exception as exc:
            errors.append(exc)
            stop.set()

    print(f"Teleoperating {len(pairs)} pair(s) (Ctrl+C to stop)...")
    threads = [
        threading.Thread(target=worker, args=pair, daemon=True) for pair in pairs
    ]
    try:
        for t in threads:
            t.start()
        while any(t.is_alive() for t in threads):
            for t in threads:
                t.join(timeout=0.2)
    except KeyboardInterrupt:
        stop.set()
        for t in threads:
            t.join(timeout=2.0)
    finally:
        stop.set()
        manager.disconnect_all()

    if errors:
        raise errors[0]
    return 0


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="Wuji Hand 1/2 SDK teleop; automatically enables matched hands, no footkey.")
    p.add_argument('--hand-model', choices=['auto', 'wujihand', 'wujihand2'], default='auto')
    p.add_argument('--side', choices=['auto', 'left', 'right', 'both'], default='auto',
                   help='auto: all complete pairs; both: require both pairs')
    for flag, default, help_ in [
        ('fps', 120., 'Loop ceiling, Hz (not a guaranteed device rate)'),
        ('hand1-effort-limit', 1.5, 'Gen-1 current limit, A'),
        ('hand1-lowpass-hz', 5., 'Gen-1 realtime LowPass cutoff, Hz'),
        ('hand2-effort-limit', 1.5, 'Gen-2 current limit, A'),
        ('hand2-kp', 5., 'Gen-2 MIT kp'),
        ('hand2-kd', .15, 'Gen-2 MIT kd'),
        ('hand2-ema', .35, 'Gen-2 EMA alpha, 0 < alpha <= 1'),
        ('stale-timeout', 1., 'Stop after missing glove frames for this many seconds'),
    ]:
        p.add_argument('--' + flag, type=float, default=default, help=help_)
    args = p.parse_args(argv)
    for key, value in vars(args).items():
        if isinstance(value, float) and (not math.isfinite(value) or value <= 0):
            p.error(f'--{key.replace("_", "-")} must be finite and > 0')
    if args.hand2_ema > 1:
        p.error('--hand2-ema must be <= 1')
    return args


def main() -> int:
    global FPS, HAND1_EFFORT_LIMIT, HAND1_LOWPASS_HZ, HAND2_EFFORT_LIMIT
    global HAND2_KP, HAND2_KD, HAND2_QPOS_EMA, STALE_TIMEOUT
    args = parse_args()
    FPS = args.fps
    HAND1_EFFORT_LIMIT, HAND1_LOWPASS_HZ = args.hand1_effort_limit, args.hand1_lowpass_hz
    HAND2_EFFORT_LIMIT, HAND2_KP, HAND2_KD = args.hand2_effort_limit, args.hand2_kp, args.hand2_kd
    HAND2_QPOS_EMA, STALE_TIMEOUT = args.hand2_ema, args.stale_timeout
    manager = SdkManager.instance()

    # Run the glove on the SDK built-in default hand URDF: the default SDK user
    # uses the built-in URDF, so switch to it before connecting and restore the
    # previous user on exit. Use a named SDK user to run with a calibrated hand
    # model.
    previous_user = use_builtin_urdf_user(manager)
    exit_code = 0
    try:
        exit_code = run_teleop(manager, args.hand_model, args.side)
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
