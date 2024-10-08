# This file contains commands to launch multiple containers of the same docker image.
First the image will need to be built. In this case a copy is currently stored at 
[this content registry](ghcr.io/chandimafernando/erobs-common-img:latest).  
Several containers need to be running for our application. Once running, clients can interact directly wiht the MoveGroup or PDF Action Server:
1. UR robot driver
2. Gripper service
3. Moveit service
4. pdf_beamtime_server (this is the sample manipulation Action server)

**TODO (ChandimaFernando)**: Change the repo link to nsls-II repo, and add CI/CD for build on version tag.

```bash
export ROBOT_IP=192.168.0.100
export UR_TYPE="ur3e"
export LAUNCH_RVIZ="false"
export DESCRIPTION_PKG="ur3e_hande_robot_description"
export DESCRIPTION_FILE="ur_with_camera_hande.xacro"
export CONFIG_PKG="ur3e_hande_moveit_config"
export CONFIG_FILE="ur.srdf"
export ROS_DISTRO=humble
export GHCR_POINTER=ghcr.io/chandimafernando/erobs-common-img:latest
```

# Enable the connection between the UR robot and the VM
```bash
podman run -it --rm --network host --ipc=host --pid=host \
    --env ROBOT_IP=$ROBOT_IP \
    --env UR_TYPE=$UR_TYPE \
    --env ROS_DISTRO=$ROS_DISTRO \
    --env DESCRIPTION_PKG=$DESCRIPTION_PKG \
    --env DESCRIPTION_FILE=$DESCRIPTION_FILE \
    ${GHCR_POINTER} \
    /bin/sh -c "printenv && . /opt/ros/${ROS_DISTRO}/setup.sh && . /root/ws/install/setup.sh && ros2 launch ur_robot_driver ur_control.launch.py ur_type:=${UR_TYPE} robot_ip:=${ROBOT_IP} description_package:=${DESCRIPTION_PKG} description_file:=${DESCRIPTION_FILE} launch_rviz:=${LAUNCH_RVIZ} tool_voltage:=24"
```

# Run gripper service to enable the gripper 
```bash
podman run -it --rm --network host --ipc=host --pid=host \
    --env ROBOT_IP=$ROBOT_IP \
    --env ROS_DISTRO=$ROS_DISTRO  \
    ${GHCR_POINTER} \
    /bin/bash -c ". /root/ws/install/setup.sh && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    ros2 run ur_robot_driver tool_communication.py --ros-args -p robot_ip:=${ROBOT_IP} & \
    sleep 5 && \
    . /root/ws/install/setup.sh && \
    ros2 run gripper_service gripper_service"
```

**NOTE**

`tool_communication.py` should not be called when `ur_control.launch.py` is called, but separately when the gripper is initiated.  The 5s delay allows socat creation before running the gripper service. 
The gripper_service has two convenience scripts, `gripper_open.cpp` and `gripper_close.cpp` that will start up client nodes to send a request to the gripper service then promptly shutdown the client nodes.

# Launch aruco_pose 
```bash
podman run -it --network host --ipc=host --pid=host \
    --env ROS_DISTRO=$ROS_DISTRO \
    ${GHCR_POINTER} \
    /bin/sh -c "printenv && \
    . /root/ws/install/setup.sh && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    ros2 launch aruco_pose aruco_pose.launch.py"
```

# Launch move_group
```bash
podman run -it --rm --network host --ipc=host --pid=host \
    --env ROBOT_IP=$ROBOT_IP \
    --env UR_TYPE=$UR_TYPE \
    --env ROS_DISTRO=$ROS_DISTRO \
    --env DESCRIPTION_PKG=$DESCRIPTION_PKG \
    --env DESCRIPTION_FILE=$DESCRIPTION_FILE \
    --env CONFIG_PKG=$CONFIG_PKG \
    --env CONFIG_FILE=$CONFIG_FILE \
    ${GHCR_POINTER} \
    /bin/sh -c "printenv && . /opt/ros/${ROS_DISTRO}/setup.sh && . /root/ws/install/setup.sh && ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=${UR_TYPE} launch_rviz:=${LAUNCH_RVIZ} description_package:=${DESCRIPTION_PKG}  launch_servo:=false description_file:=${DESCRIPTION_FILE} moveit_config_package:=${CONFIG_PKG} moveit_config_file:=${CONFIG_FILE}"
```

# Launch the pdf_beamtime_server
```bash
podman run -it --network host --ipc=host --pid=host \
    --env ROS_DISTRO=$ROS_DISTRO \
    ${GHCR_POINTER} \
    /bin/sh -c "printenv && . /opt/ros/${ROS_DISTRO}/setup.sh && . /root/ws/install/setup.sh && ros2 launch pdf_beamtime pdf_beamtime.launch.py"
```

# Launch the pdf_beamtime_fidpose_server
```bash
podman run -it --network host --ipc=host --pid=host \
    --env ROS_DISTRO=$ROS_DISTRO \
    ${GHCR_POINTER} \
    /bin/sh -c "printenv && . /opt/ros/${ROS_DISTRO}/setup.sh && . /root/ws/install/setup.sh && ros2 launch pdf_beamtime pdf_beamtime_fidpose_server.launch.py"
```

# Test base rotation for Emergency stop
```bash
podman run -it --network host --ipc=host --pid=host \
    --env ROS_DISTRO=$ROS_DISTRO \
    ${GHCR_POINTER} \
    /bin/sh -c "printenv && . /opt/ros/${ROS_DISTRO}/setup.sh && . /root/ws/install/setup.sh && ros2 action send_goal pdf_beamtime_action_server pdf_beamtime_interfaces/action/PickPlaceControlMsg '{pickup_approach: [1.571, -1.571, 0.0, -1.571, 0.0, 0.0]}'"
```

# Cancel the base rotation test without the emergency stop
```bash
podman run -it --network host --ipc=host --pid=host \
    --env ROS_DISTRO=$ROS_DISTRO \
    ${GHCR_POINTER} \
    /bin/sh -c "printenv && . /opt/ros/${ROS_DISTRO}/setup.sh && . /root/ws/install/setup.sh && ros2 action send_goal pdf_beamtime_action_server pdf_beamtime_interfaces/action/PickPlaceControlMsg {pickup_approach: [1.571, -1.571, 0.0, -1.571, 0.0, 0.0,]} --cancel"
```
