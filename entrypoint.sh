#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
if [[ "${1:-}" == camera ]]; then
    : "${CAMERA_SERIAL:?Set CAMERA_SERIAL to the connected Femto Bolt serial number}"
    exec ros2 launch /opt/femto/camera.launch.py "serial_number:=$CAMERA_SERIAL"
fi
exec "$@"
