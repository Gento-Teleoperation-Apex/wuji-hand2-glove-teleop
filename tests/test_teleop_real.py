"""Generation-specific SDK parameters, pairing and failure cleanup (mock devices)."""
import contextlib
import importlib.util
import io
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

sdk = MagicMock()
sdk.WujiHand2 = type('WujiHand2', (), {})
sdk.JointCommand = lambda p, v, e: NS(position=p)
with patch.dict(sys.modules, {'wuji_sdk': sdk}):
    spec = importlib.util.spec_from_file_location('real', Path(__file__).resolve().parents[1] /
                                                 'examples/python/retargeting/1.teleop_real.py')
    real = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(real)


class TeleopTests(unittest.TestCase):
    def test_generation_defaults(self):
        args = real.parse_args([])
        self.assertEqual((args.hand1_effort_limit, args.hand1_lowpass_hz), (1.5, 5.))
        self.assertEqual((args.hand2_effort_limit, args.hand2_kp, args.hand2_kd, args.hand2_ema),
                         (1.5, 5., .15, .35))

    def test_invalid_parameters_fail_before_discovery(self):
        for argv in [['--hand2-ema', '1.1'], ['--hand1-lowpass-hz', 'nan'],
                     ['--hand2-kp', 'inf'], ['--fps', '0'], ['--hand1-effort-limit', '-1']]:
            with self.subTest(argv=argv), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                real.parse_args(argv)

    def test_gen1_uses_lowpass_and_current_without_gen2_mit(self):
        hand = MagicMock(serial_number='H1')
        with patch.object(real, 'teleop') as teleop, patch.object(real, 'LowPass') as lowpass:
            real.run_pair(MagicMock(serial_number='G1'), hand, 'right', threading.Event())
        hand.set_all_effort_limit.assert_called_once_with(1.5)
        hand.mit_params.assert_not_called()
        hand.enable.assert_called_once()
        hand.disable.assert_called_once()
        lowpass.assert_called_once_with(cutoff_hz=5.)
        self.assertEqual(teleop.call_args.args[1], real.HandModel.WujiHand)

    def test_gen2_uses_mit_and_ema(self):
        hand = MagicMock(spec=sdk.WujiHand2)
        for name in ['clear_fault', 'effort_limit', 'mit_params', 'enable', 'disable', 'joint_command']:
            setattr(hand, name, MagicMock())
        hand.serial_number = 'H2'
        with patch.object(real, 'teleop') as teleop, patch.object(real, 'wait_hand2_enabled', return_value=True):
            real.run_pair(MagicMock(serial_number='G1'), hand, 'right', threading.Event())
        hand.effort_limit.return_value.set.assert_called_once_with(1.5)
        hand.mit_params.return_value.set.assert_called_once_with((5., .15))
        self.assertEqual(teleop.call_args.args[1], real.HandModel.WujiHand2)
        self.assertTrue(teleop.call_args.kwargs['smooth_qpos'])
        hand.disable.assert_called_once()

    def test_enable_failure_disables_hand(self):
        hand = MagicMock(serial_number='H1')
        hand.enable.side_effect = RuntimeError('enable failed')
        with self.assertRaisesRegex(RuntimeError, 'enable failed'):
            real.run_pair(MagicMock(serial_number='G1'), hand, 'right', threading.Event())
        hand.disable.assert_called_once()

    def devices(self, extra_glove=False):
        manager = MagicMock()
        hand = MagicMock(serial_number='H1')
        hand.handedness_name.return_value = 'right'
        glove = MagicMock(serial_number='G1')
        glove.hand_side.return_value.get.return_value = 'right'
        found = [NS(sn='G1', device_type=real.DeviceType.WujiGlove, address='192.168.1.100:50001'),
                 NS(sn='H1', device_type=real.DeviceType.WujiHand, address='usb:H1'),
                 NS(sn='H2', device_type=real.DeviceType.WujiHand2, address='6.6.7.110:7447')]
        by_sn = {'G1': glove, 'H1': hand}
        if extra_glove:
            other = MagicMock(serial_number='G2')
            other.hand_side.return_value.get.return_value = 'right'
            by_sn['G2'] = other
            found.append(NS(sn='G2', device_type=real.DeviceType.WujiGlove, address='192.168.1.101:50001'))
        manager.scan.return_value = found
        manager.connect.side_effect = lambda **kw: by_sn[kw['sn']]
        return manager, hand, glove

    def test_gen1_selection_excludes_ethernet_gen2(self):
        manager, hand, glove = self.devices()
        with patch.object(real, 'run_pair') as run:
            self.assertEqual(real.run_teleop(manager, 'wujihand'), 0)
        self.assertEqual([c.kwargs['sn'] for c in manager.connect.call_args_list], ['G1', 'H1'])
        self.assertEqual(run.call_args.args[:3], (glove, hand, 'right'))

    def test_both_requires_both_pairs_before_enabling(self):
        manager, hand, _ = self.devices()
        with patch.object(real, 'run_pair') as run, self.assertRaisesRegex(RuntimeError, 'Missing requested left'):
            real.run_teleop(manager, 'wujihand', 'both')
        run.assert_not_called()
        hand.enable.assert_not_called()

    def test_duplicate_side_rejected_before_enabling(self):
        manager, hand, _ = self.devices(extra_glove=True)
        with patch.object(real, 'run_pair') as run, self.assertRaisesRegex(RuntimeError, 'Multiple right gloves'):
            real.run_teleop(manager, 'wujihand')
        run.assert_not_called()
        hand.enable.assert_not_called()

    def test_stale_skeleton_stops(self):
        with patch.object(real, 'read_keypoints', return_value=None), \
                patch.object(real.time, 'monotonic', side_effect=[0., 2., 2.]), \
                self.assertRaisesRegex(RuntimeError, 'no fresh glove skeleton'):
            real.teleop(MagicMock(), real.HandModel.WujiHand, MagicMock(), 'right', threading.Event())

    def test_invalid_output_is_not_sent(self):
        send = MagicMock()
        with patch.object(real, 'read_keypoints', return_value=np.zeros((21, 3))), \
                patch.object(real.RetargetSession, 'for_hand') as session:
            session.return_value.step.return_value = [float('nan')] * 20
            with self.assertRaisesRegex(ValueError, 'retarget'):
                real.teleop(MagicMock(), real.HandModel.WujiHand, send, 'right', threading.Event())
        send.assert_not_called()


if __name__ == '__main__':
    unittest.main()
