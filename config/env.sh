#!/usr/bin/env bash
# SDK commands also work without ROS. Keep deployment overrides outside package files.
if [[ -f /etc/apex/apex.env ]]; then
  set -a
  source /etc/apex/apex.env
  set +a
fi
if [[ -f /opt/ros/humble/setup.bash ]]; then source /opt/ros/humble/setup.bash; fi
if [[ -f /opt/kernelmind/apex/install/setup.bash ]]; then source /opt/kernelmind/apex/install/setup.bash; fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
if [[ ${APEX_DDS:-} == cyclone ]]; then export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; fi
if [[ ${APEX_DDS:-} == fastdds || ${APEX_DDS:-} == fastrtps ]]; then export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; fi
if [[ $RMW_IMPLEMENTATION == rmw_cyclonedds_cpp && -f /etc/apex/cyclonedds.xml ]]; then export CYCLONEDDS_URI=file:///etc/apex/cyclonedds.xml; fi
export WUJI_CANONICAL_ONLY=1
export WUJI_APEX_STATE_TOPIC="/${APEX_ROS_NAMESPACE:+${APEX_ROS_NAMESPACE#/}/}control/switch_state"
export WUJI_PLAYBACK_KEY_TOPIC="/${APEX_ROS_NAMESPACE:+${APEX_ROS_NAMESPACE#/}/}playback_key"
# Preserve existing standalone pairings when upgrading this deployment.
if [[ -f /opt/kernelmind/wuji-hand2/config/pairing.env ]]; then source /opt/kernelmind/wuji-hand2/config/pairing.env; fi
if [[ -f /etc/wuji-hand2/config.env ]]; then source /etc/wuji-hand2/config.env; fi
if [[ -f ${XDG_CONFIG_HOME:-$HOME/.config}/wuji-hand2/config.env ]]; then source "${XDG_CONFIG_HOME:-$HOME/.config}/wuji-hand2/config.env"; fi
if [[ -z ${WUJI_PYTHON:-} ]]; then
  if [[ -x $HOME/.venvs/wuji/bin/python ]]; then export WUJI_PYTHON="$HOME/.venvs/wuji/bin/python";
  else export WUJI_PYTHON=python3; fi
fi
