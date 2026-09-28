#!/usr/bin/env bash
set -euo pipefail
version="${1:-2.2.0}"
arch="${2:-arm64}"
[[ $version =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]] || { echo 'Version must be X.Y.Z'; exit 2; }
[[ $arch == arm64 || $arch == amd64 || $arch == all ]] || { echo 'Architecture must be arm64/amd64/all'; exit 2; }
root="$(cd -- "$(dirname -- "$0")" && pwd)"
mkdir -p "$root/dist" "$root/build"
stage="$(mktemp -d "$root/build/deb.XXXXXX")"
# Unique staging paths make parallel builds safe; retain staging for inspection.
app="$stage/opt/kernelmind/wuji-hand2"
mkdir -p "$app" "$stage/DEBIAN" "$stage/usr/bin" "$stage/etc/wuji-hand2" "$stage/usr/lib/systemd/user" "$stage/usr/share/doc/wuji-hand2-glove-teleop"
for dir in retargeting wuji_hand_2 wuji_glove; do
 mkdir -p "$app/$dir"
 for f in "$root/examples/python/$dir/"*.py; do install -m 644 "$f" "$app/$dir/"; done
done
for dir in tools config integration pose_tools pose_web; do
 mkdir -p "$app/$dir"
 for f in "$root/$dir/"*; do
   [[ -f "$f" ]] || continue
   case "$(basename "$f")" in test_*|*.log|*_results.json) continue ;; esac
   install -m 644 "$f" "$app/$dir/"
 done
done
install -m 644 "$root/config/config.env" "$stage/etc/wuji-hand2/config.env"
install -m 644 "$root/packaging/wuji-pose-web.service" "$stage/usr/lib/systemd/user/"
install -m 644 "$root/README.md" "$root/CHANGELOG.md" "$root/LICENSE" "$stage/usr/share/doc/wuji-hand2-glove-teleop/"
cp -r "$root/docs" "$stage/usr/share/doc/wuji-hand2-glove-teleop/"
install -m 644 "$root/README.md" "$root/LICENSE" "$app/"
for name in wuji-setup-runtime wuji-hand2-check wuji-hand2-sdk wuji-hand2-tuned wuji-hand2-glove wuji-hand2-driver wuji-hand2-bag wuji-both-record wuji-left-record wuji-right-record wuji-both-poses wuji-left-poses wuji-right-poses wuji-pose-web wujihand2-teleop wujihand2-teleop-real wujihand2-ros-driver wujihand2-save-home wujihand2-fingertip; do
 install -m 755 "$root/packaging/launcher" "$stage/usr/bin/$name"
done
sed -i 's|/usr/local/bin/wuji-pose-web|/usr/bin/wuji-pose-web|' "$stage/usr/lib/systemd/user/wuji-pose-web.service"
cat > "$stage/DEBIAN/control" <<CONTROL
Package: wuji-hand2-glove-teleop
Version: $version-1
Section: science
Priority: optional
Architecture: $arch
Depends: python3 (>= 3.10), python3-numpy, python3-venv, bash
Recommends: ros-humble-rclpy, ros-humble-sensor-msgs, ros-humble-std-msgs, ros-humble-std-srvs, ros-humble-rosbag2-py, ros-humble-ros2bag, ros-humble-rosbag2-storage-mcap, ros-humble-rmw-fastrtps-cpp
Maintainer: Gento Teleoperation Apex <Gento-Teleoperation-Apex@users.noreply.github.com>
Homepage: https://github.com/Gento-Teleoperation-Apex/wuji-hand2-glove-teleop
Description: Wuji Hand2 dual glove SDK and ROS teleoperation suite
 Connection checks, SDK baseline and tuned control, F7 or no-footkey,
 ROS driver, manual teaching, independent pose selection, web UI and bags.
 Requires external wuji-sdk 2026.8.31; setup command provided. No autostart.
CONTROL
printf '/etc/wuji-hand2/config.env\n' > "$stage/DEBIAN/conffiles"
install -m 755 "$root/packaging/postinst" "$stage/DEBIAN/postinst"
(cd "$stage" && find opt usr etc -type f -print0 | sort -z | xargs -0 md5sum) > "$stage/DEBIAN/md5sums"
dpkg-deb --root-owner-group --build "$stage" "$root/dist/wuji-hand2-glove-teleop_${version}-1_${arch}.deb"
sha256sum "$root/dist/wuji-hand2-glove-teleop_${version}-1_${arch}.deb"
