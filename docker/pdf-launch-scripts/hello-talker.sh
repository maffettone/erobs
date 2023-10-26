#!/bin/bash

# Launch a trivial talker node using the ur base
ROS_DISTRO=humble

docker run -it --network host ghcr.io/nsls2/erobs-ur-driver:latest \
       /bin/sh -c ". /opt/ros/${ROS_DISTRO}/setup.sh && ros2 run demo_nodes_cpp talker"
