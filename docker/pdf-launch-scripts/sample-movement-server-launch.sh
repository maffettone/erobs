#!/bin/bash
# Launch Docker container for ur-driver. TO BE DEPRECATED.
ROS_DISTRO=humble
UR_TYPE="ur3e"
ROBOT_IP=10.66.218.141
REVERSE_IP=10.66.219.39

docker run -it --network host --ipc=host --pid=host\
    ghcr.io/nsls2/ur-hande-draft:latest \
    /bin/bash -c ". /root/ws/install/setup.sh && \
ros2 launch sample_movement_ur3e_sim sample_movement_ur3e_sim.launch.py"
