#!/bin/bash
set -e

# Source the ROS setup.bash file
source "/opt/ros/noetic/setup.bash"

# Source the catkin workspace setup file
if [ -f "/root/catkin_ws/devel/setup.bash" ]; then
    source "/root/catkin_ws/devel/setup.bash"
fi

exec "$@"
