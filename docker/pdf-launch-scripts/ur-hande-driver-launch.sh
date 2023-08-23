#!/bin/bash
# Launch Docker container for ur-driver
ROS_DISTRO=humble
UR_TYPE="ur3e"
ROBOT_IP=10.66.218.141
REVERSE_IP=10.66.219.39

docker run -it --network host\
    --env ROBOT_IP=$ROBOT_IP \
    --env REVERSE_IP=$REVERSE_IP \
    ghcr.io/nsls2/ur-hande-draft:latest \
    /bin/bash -c ". /root/ws/ERoBS_ur3e/install/setup.sh && \
ros2 launch ur_hande_moveit_config ur_control_hande.launch.py \
ur_type:=${UR_TYPE} robot_ip:=${ROBOT_IP} launch_rviz:=false use_tool_communication:=true"
