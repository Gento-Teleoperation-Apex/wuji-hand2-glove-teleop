import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace as NS
from unittest.mock import MagicMock,patch
import numpy as np
spec=importlib.util.spec_from_file_location('check_devices',Path(__file__).resolve().parents[1]/'tools/check_devices.py')
check=importlib.util.module_from_spec(spec);spec.loader.exec_module(check)
class CheckTests(unittest.TestCase):
    def test_read_only_all_and_missing_pair(self):
        sdk=MagicMock();manager=sdk.SdkManager.instance.return_value
        devices={};found=[];env={}
        for side in ['left','right']:
            for kind in ['hand','glove']:
                sn=f'{side}-{kind}';dev=MagicMock();dev.serial_number=sn
                dev.handedness.return_value.get.return_value=side;dev.hand_side.return_value.get.return_value=side
                dev.online_joints_count.return_value.get.return_value=20
                dev.joint_states.return_value.subscribe.return_value.recv.return_value=NS(joints=[NS(nid=b*5+j+1,position=.1) for b in range(5) for j in range(4)])
                dev.hand_skeleton.return_value.subscribe.return_value.recv.return_value=NS(joints=[NS(pose=NS(position=[.1,.2,.3])) for _ in range(21)])
                devices[sn]=dev;found.append(NS(sn=sn,address='mock',device_type=sdk.DeviceType.WujiHand2 if kind=='hand' else sdk.DeviceType.WujiGlove))
                key='WUJI_'+side.upper()+('_GLOVE' if kind=='glove' else '')
                env[key+'_SN']=sn;env[key+'_ADDRESS']=''
        manager.scan.return_value=found;manager.connect.side_effect=lambda **kw:devices[kw['sn']]
        sdk.RetargetSession.for_hand.return_value.step.return_value=np.zeros(20)
        with patch.dict(sys.modules,{'wuji_sdk':sdk}),patch.dict(os.environ,env),patch.object(sys,'argv',['check','--seconds','.2']):
            out=io.StringIO()
            with contextlib.redirect_stdout(out):self.assertEqual(check.main(),0)
            report=json.loads(out.getvalue());self.assertEqual(len(report['checks']),4);self.assertFalse(report['errors'])
            manager.scan.return_value=found[:-1]
            with contextlib.redirect_stdout(io.StringIO()):self.assertEqual(check.main(),1)
        for dev in devices.values():
            dev.enable.assert_not_called();dev.disable.assert_not_called();dev.joint_command.assert_not_called();dev.tactile_calibrate.assert_not_called()
if __name__=='__main__':unittest.main()
