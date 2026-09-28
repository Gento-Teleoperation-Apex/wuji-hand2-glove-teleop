import importlib.util
import os
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace as NS
from unittest.mock import MagicMock,patch
import numpy as np

root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'examples/python/retargeting'))
sdk=MagicMock();sdk.JointCommand=lambda p,v,e:NS(position=p)
sys.modules['wuji_sdk']=sdk
spec=importlib.util.spec_from_file_location('tuned',root/'examples/python/retargeting/2.teleop_tuned.py')
tuned=importlib.util.module_from_spec(spec);spec.loader.exec_module(tuned)

class DirectTests(unittest.TestCase):
    def setUp(self):
        self.env=patch.dict(os.environ,{'WUJI_RIGHT_SN':'HAND-R','WUJI_LEFT_SN':'HAND-L'},clear=False);self.env.start()
        self.hand=MagicMock();self.hand.serial_number='HAND-R';self.hand.handedness.return_value.get.return_value='right'
        self.manager=MagicMock();self.manager.scan.return_value=[NS(sn='HAND-R',device_type=sdk.DeviceType.WujiHand2)]
        self.manager.connect.return_value=self.hand
        frame=NS(joints=[NS(nid=b*5+j+1,position=.1) for b in range(5) for j in range(4)])
        self.hand.joint_states.return_value.subscribe.return_value.recv.return_value=frame
        self.pub=self.hand.joint_command.return_value.publish.return_value
    def tearDown(self):self.env.stop()
    def driver(self):return tuned.Hand2DirectDriver(self.manager,['right'],{'right':'hand_right'},ros_bridge=False)
    def test_gate_enable_send_cleanup(self):
        d=self.driver();self.hand.enable.assert_not_called()
        d.send([.2]*20,'hand_right');self.pub.send.assert_not_called()
        d.set_footkey(True)
        with patch.object(tuned,'_wait_hand2_enabled',return_value=True):d.send([.2]*20,'hand_right')
        self.hand.enable.assert_called_once();self.assertEqual(self.pub.send.call_count,2)
        np.testing.assert_allclose([j.position for j in self.pub.send.call_args.args[0]],[.135]*20)
        d.set_footkey(False);d.send([.9]*20,'hand_right');self.assertEqual(self.pub.send.call_count,3)
        np.testing.assert_allclose([j.position for j in self.pub.send.call_args.args[0]],[.135]*20)
        d.close();self.hand.disable.assert_called_once()
    def test_invalid_target_never_enables(self):
        d=self.driver();d.set_footkey(True)
        for q in [[0.]*19,[float('nan')]*20]:
            with self.assertRaises(ValueError):d.send(q,'hand_right')
        self.hand.enable.assert_not_called();d.close();self.hand.disable.assert_not_called()
    def test_missing_pair_never_enables(self):
        with self.assertRaises(RuntimeError):tuned.Hand2DirectDriver(self.manager,['left','right'],{'left':'hand_left','right':'hand_right'},ros_bridge=False)
        self.hand.enable.assert_not_called()
    def test_identity_rejected(self):
        self.hand.serial_number='WRONG'
        with self.assertRaises(RuntimeError):self.driver()
        self.hand.enable.assert_not_called()
    def test_failed_enable_cleaned(self):
        d=self.driver();d.set_footkey(True)
        with patch.object(tuned,'_wait_hand2_enabled',return_value=False):
            with self.assertRaises(RuntimeError):d.send([.2]*20,'hand_right')
        d.close();self.hand.disable.assert_called_once()
    def test_official_profile_neutral_parameters(self):
        args=NS(profile='official',side='right',hand_model='wujihand2',drive='sdk',hand_name=None,no_home=True,no_footkey=True,go_home_service='unused',go_home_duration=1.,opposition_enable_distance=.1)
        with patch.object(tuned,'parse_args',return_value=args),patch.object(tuned,'select_sdk_user') as select,patch.object(tuned,'run_teleop',return_value=0) as run:
            self.assertEqual(tuned.main(),0)
        c=run.call_args.args;np.testing.assert_equal(c[5],np.ones(5));self.assertEqual(c[6:11],(1.,0.,.1,0.,0.))
        self.assertTrue(args.default_user)
    def test_neutral_transform_preserves_sdk_output(self):
        q=np.arange(20,dtype=np.float32)*.02
        np.testing.assert_array_equal(tuned.tune_qpos(q,1,0,None,0,0),q)

if __name__=='__main__':unittest.main()
