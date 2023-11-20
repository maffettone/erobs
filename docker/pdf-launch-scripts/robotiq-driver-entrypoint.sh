#!/bin/bash

export ROBOT_IP=$ROBOT_IP
export REVERSE_IP=$REVERSE_IP


. /root/ws/install/setup.sh && ros2 run ur_robot_driver tool_communication.py --ros-args -p robot_ip:=${ROBOT_IP} &
sleep 10
. /root/ws/install/setup.sh && ros2 launch robotiq_driver tests.launch.py
