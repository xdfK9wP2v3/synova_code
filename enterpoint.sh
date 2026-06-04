#!/bin/bash
set -e

# setup ros2 environment
/usr/sbin/service ssh restart
source "/opt/ros/$ROS_DISTRO/setup.bash" --
source "/ros_workspace/install/setup.bash" --

ip route del default
ip route add default via 192.168.${SUBNET}.1

exec "$@"
