#!/bin/bash

echo "Starting custom program ... "
. /opt/ros/${ROS_DISTRO}/setup.bash
. install/setup.bash 
# ros2 run hello_moveit hello_moveit
ro2 launch hello_moveit pick_place_repeat.launch.py
