#!/bin/zsh
set -e

ros_env_setup="/opt/ros/$ROS_DISTRO/setup.zsh"
echo "sourcing   $ros_env_setup"
source "$ros_env_setup"

echo "ROS_ROOT   $ROS_ROOT"
echo "ROS_DISTRO $ROS_DISTRO"

echo "Activating VNC SERVER"
#./data/scripts/Robot_launch_global_F4_0.05

exec "$@"

