#!/bin/bash
# Launch Docker container for ur-driver
ROS_DISTRO=humble
UR_TYPE="ur3e"
ROBOT_IP=10.66.218.141
REVERSE_IP=10.66.219.39

docker run -it \
    --env ROBOT_IP=$ROBOT_IP \
    --env REVERSE_IP=$REVERSE_IP \
    ghcr.io/nsls2/erobs-ur-driver:latest \
    /bin/sh -c "printenv && . /opt/ros/${ROS_DISTRO}/setup.sh && ros2 launch ur_robot_driver ur_control.launch.py ur_type:=${UR_TYPE} robot_ip:=${ROBOT_IP} launch_rviz:=false"
