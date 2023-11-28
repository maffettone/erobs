#!/bin/bash

echo "Starting VNC server..."
# Setup VNC server
Xvfb :1 -screen 0 2560x1440x16 &
x11vnc -rfbport 5901 -bg -quiet -forever -shared -display :1


echo "Starting rviz and moveit with ur_type ${UR_TYPE}..."
.  /opt/ros/${ROS_DISTRO}/setup.bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=${UR_TYPE} launch_rviz:=${LAUNCH_RVIZ} description_package:=${DESCRIPTION_PKG} moveit_config_package:=${CONFIG_PKG} moveit_config_file:=${CONFIG_FILE}