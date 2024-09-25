#!/bin/bash
set -e

# Setup ROS environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/root/catkin_ws/devel/setup.bash"

# Execute the command passed to docker run
exec "$@"
