#!/bin/bash

set -e

source "/opt/ros/$ROS_DISTRO/setup.bash"
source /root/ros2_ws/install/local_setup.bash

envsubst < /root/ros2_ws/src/bridge.yaml > new_bridge.yaml
mv new_bridge.yaml /root/ros2_ws/src/bridge.yaml

exec "$@"
