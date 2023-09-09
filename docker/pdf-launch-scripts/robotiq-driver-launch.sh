#!/bin/bash
# Driver launch for Gripper Tool (Robotiq HandE)
ROS_DISTRO=humble
UR_TYPE="ur3e"
ROBOT_IP=10.66.218.141
REVERSE_IP=10.66.218.39

docker run -it --network host --ipc=host --pid=host\
    --env ROBOT_IP=$ROBOT_IP \
    --env REVERSE_IP=$REVERSE_IP \
    -v ./robotiq-driver-entrypoint.sh:/root/entrypoints/robotiq-driver-entrypoint.sh \
    ghcr.io/nsls2/ur-hande-draft:latest \
    /bin/bash /root/entrypoints/robotiq-driver-entrypoint.sh
