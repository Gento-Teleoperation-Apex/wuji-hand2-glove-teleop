#!/usr/bin/env bash
set -euo pipefail
# Run as the normal control user. ROS Humble requires system Python 3.10 on Jammy.
venv="${WUJI_VENV:-$HOME/.venvs/wuji}"
python3 -m venv --system-site-packages "$venv"
"$venv/bin/python" -m pip install 'wuji-sdk==2026.8.31' 'numpy>=1.21,<2' 'pynput==1.7.7'
"$venv/bin/python" -c 'from wuji_sdk import SdkManager, RetargetSession; import numpy; print("SDK imports OK")'
printf 'Runtime ready: %s/bin/python\nNo devices were connected or enabled.\n' "$venv"
