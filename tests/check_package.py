"""Verify staged deb payload, launchers and conffile/pose preservation contract."""
from pathlib import Path
import hashlib
import subprocess
import sys
import tempfile

for arg in sys.argv[1:]:
    deb=Path(arg).resolve()
    with tempfile.TemporaryDirectory() as tmp:
        stage=Path(tmp);subprocess.run(['dpkg-deb','-R',str(deb),str(stage)],check=True)
        app=stage/'opt/kernelmind/wuji-hand2'
        for name in ['retargeting/2.teleop_tuned.py','retargeting/wujihand2_ros_driver.py','tools/check_devices.py','tools/hand_bag.py','pose_tools/record_poses.py','pose_tools/play_poses.py','pose_tools/pose_common.py','pose_web/server.py','pose_web/index.html']:
            assert (app/name).is_file(),name
        assert (stage/'DEBIAN/conffiles').read_text().strip()=='/etc/wuji-hand2/config.env'
        assert not list(stage.glob('home/**/*')) and not list(app.rglob('*poses.json'))
        assert not (app/'config/pairing.env').exists()
        for line in (stage/'DEBIAN/md5sums').read_text().splitlines():
            digest,path=line.split(maxsplit=1)
            assert hashlib.md5((stage/path).read_bytes()).hexdigest()==digest,path
        for wrapper in (stage/'usr/bin').iterdir():
            assert wrapper.stat().st_mode & 0o111
            subprocess.run(['bash','-n',str(wrapper)],check=True)
        for py in app.rglob('*.py'):compile(py.read_bytes(),str(py),'exec')
        assert 'systemctl start' not in (stage/'DEBIAN/postinst').read_text()
        assert 'enable()' not in (stage/'DEBIAN/postinst').read_text()
        print('PASS:',deb.name,'payload, checksums, shell/Python syntax, no user poses, preserved pairing, no autostart')
