#!/usr/bin/env bash
set -eo pipefail
root="$(cd -- "$(dirname -- "$0")/.." && pwd)"
/usr/bin/python3 "$root/tests/test_sdk_direct.py"
/usr/bin/python3 "$root/tests/test_connection_check.py"
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI
export ROS_DOMAIN_ID=224
/usr/bin/python3 "$root/pose_tools/test_dual_poses.py"
/usr/bin/python3 "$root/pose_tools/test_manual_teach.py"
/usr/bin/python3 "$root/pose_tools/test_ros_poses.py"
export ROS_DOMAIN_ID=225
/usr/bin/python3 "$root/pose_web/test_web_ros.py"

bash "$root/tests/run_bag_test.sh"
