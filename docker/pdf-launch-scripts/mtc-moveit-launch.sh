#!/bin/bash
# Launch Docker container for ur-driver
ROS_DISTRO=humble
UR_TYPE="ur3e"
ROBOT_IP=10.66.218.141
REVERSE_IP=10.66.219.39

# Turn off tool communication here? 
docker run -it --network host --ipc=host --pid=host\
    ghcr.io/nsls2/ur-hande-draft:latest \
    /bin/bash -c ". /root/ws/install/setup.sh && \
ros2 launch ur_hande_moveit_config mtc_moveit_rviz.launch.py \
launch_rviz:=false use_tool_communication:=true"
