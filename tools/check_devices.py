#!/usr/bin/env python3
"""Read-only discovery, paired identity, complete hand feedback and glove retarget check."""
import argparse
import json
import os
from pathlib import Path
import sys
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scan',action='store_true',help='Only list discovered devices')
    p.add_argument('--side',choices=['left','right','both'],default='both')
    p.add_argument('--kind',choices=['hands','gloves','all'],default='all')
    p.add_argument('--seconds',type=float,default=3.)
    p.add_argument('--output',type=Path)
    args=p.parse_args()
    if not 0.2 <= args.seconds <= 60:p.error('--seconds must be 0.2..60')
    from wuji_sdk import SdkManager, DeviceType, ConnectOptions, RetargetSession, HandModel, Handedness
    import numpy as np
    manager=SdkManager.instance();report={'read_only':True,'devices':[],'checks':[],'errors':[]}
    try:
        found=manager.scan()
        report['devices']=[dict(sn=d.sn,type=str(d.device_type),address=d.address) for d in found]
        if not args.scan:
            sides=['left','right'] if args.side=='both' else [args.side]
            kinds=['hands','gloves'] if args.kind=='all' else [args.kind]
            for kind in kinds:
                dtype=DeviceType.WujiHand2 if kind=='hands' else DeviceType.WujiGlove
                for side in sides:
                    key=f'WUJI_{side.upper()}'+('_GLOVE' if kind=='gloves' else '')
                    sn=os.getenv(key+'_SN');address=os.getenv(key+'_ADDRESS')
                    options=ConnectOptions(timeout_ms=2500,retry_count=0,enable_bridge=False,auto_time_sync_interval_ms=None)
                    candidates=[d for d in found if d.device_type==dtype and (not sn or d.sn==sn)]
                    try:
                        matches=[]
                        for d in candidates:
                            dev=manager.connect(sn=d.sn,device_name=f'check_{kind}_{d.sn}',options=options)
                            if dev.serial_number!=d.sn:raise ValueError('Identity mismatch; check duplicate IPs')
                            label=str(dev.handedness().get() if kind=='hands' else dev.hand_side().get()).lower()
                            if side in label:matches.append((dev,d.address))
                        if not matches and sn and address:
                            dev=manager.connect(address=address,device_name=f'check_{kind}_{side}',options=options)
                            label=str(dev.handedness().get() if kind=='hands' else dev.hand_side().get()).lower()
                            if dev.serial_number!=sn or side not in label:raise ValueError('Configured address identity/side mismatch')
                            matches=[(dev,address)]
                        if len(matches)!=1:raise ValueError(f'Expected one {side} {kind}; found {len(matches)}; check power/network/pairing')
                        dev,live=matches[0];row=dict(kind=kind,side=side,sn=dev.serial_number,address=live,frames=0)
                        if kind=='hands':
                            row['online_joints']=dev.online_joints_count().get()
                            sub=dev.joint_states().subscribe()
                        else:
                            sub=dev.hand_skeleton().subscribe()
                            session=RetargetSession.for_hand(HandModel.WujiHand2,side=Handedness.Left if side=='left' else Handedness.Right)
                            row['retarget_frames']=0
                        deadline=time.monotonic()+args.seconds
                        try:
                            while time.monotonic()<deadline:
                                frame=sub.recv()
                                if frame is None:time.sleep(.002);continue
                                if kind=='hands':
                                    expected={b*5+j+1 for b in range(5) for j in range(4)}
                                    if {j.nid for j in frame.joints}==expected and all(np.isfinite(j.position) for j in frame.joints):row['frames']+=1
                                else:
                                    kp=np.asarray([j.pose.position for j in frame.joints],dtype=np.float32)
                                    if kp.shape!=(21,3) or not np.isfinite(kp).all():continue
                                    row['frames']+=1;q=np.asarray(session.step(kp))
                                    if q.shape==(20,) and np.isfinite(q).all():row['retarget_frames']+=1
                        finally:sub.close()
                        row['ok']=row['frames']>0 and (row['online_joints']==20 if kind=='hands' else row['retarget_frames']>0)
                        report['checks'].append(row)
                        if not row['ok']:raise ValueError('No complete valid feedback/retarget output')
                    except Exception as exc:report['errors'].append(f'{side} {kind}: {exc}')
                    finally:manager.disconnect_all()
    except Exception as exc:report['errors'].append(str(exc))
    finally:manager.disconnect_all()
    data=json.dumps(report,ensure_ascii=False,indent=2);print(data)
    if args.output:args.output.write_text(data+'\n')
    return int(bool(report['errors']))

if __name__=='__main__':sys.exit(main())
