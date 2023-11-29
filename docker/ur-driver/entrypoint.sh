#!/bin/bash

echo "Connecting to simulator through ursim for ur_type ${UR_TYPE}..."
.  /opt/ros/${ROS_DISTRO}/setup.bash
.  /root/ws/install/setup.bash 

ros2 launch ur_robot_driver ur_control.launch.py ur_type:=${UR_TYPE} robot_ip:=${ROBOT_IP} launch_rviz:=false description_package:=${DESCRIPTION_PKG} description_file:="hande_extension.urdf.xacro"
