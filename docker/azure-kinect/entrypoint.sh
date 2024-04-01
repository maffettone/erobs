#!/bin/bash

echo "Starting VNC server..."
# Setup VNC server
Xvfb :1 -screen 0 2560x1440x16 &
x11vnc -rfbport 5905 -bg -quiet -forever -shared -display :1

echo "source the workspace"
.  /opt/ros/${ROS_DISTRO}/setup.bash
. /root/ws/install/setup.bash

echo "run driver.launch"
ros2 launch azure_kinect_ros_driver driver.launch.py
