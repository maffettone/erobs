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
export ROBOT_IP=10.67.218.141
export UR_TYPE="ur3e"
export LAUNCH_RVIZ="false"
export DESCRIPTION_PKG="ur3e_hande_robot_description"
export DESCRIPTION_FILE="ur_with_hande.xacro"
export CONFIG_PKG="ur3e_hande_moveit_config"
export CONFIG_FILE="ur.srdf"
export ROS_DISTRO=humble
export GHCR_POINTER=ghcr.io/chandimafernando/erobs-common-img:latest
```

# Enable the connection between the UR robot and the VM
```bash
podman run -it --network host --ipc=host --pid=host \
    --env ROBOT_IP=$ROBOT_IP \
    --env UR_TYPE=$UR_TYPE \
    --env ROS_DISTRO=$ROS_DISTRO \
    --env DESCRIPTION_PKG=$DESCRIPTION_PKG \
    --env DESCRIPTION_FILE=$DESCRIPTION_FILE \
    ${GHCR_POINTER} \
    /bin/sh -c "printenv && . /opt/ros/${ROS_DISTRO}/setup.sh && . /root/ws/install/setup.sh && ros2 launch ur_robot_driver ur_control.launch.py ur_type:=${UR_TYPE} robot_ip:=${ROBOT_IP} description_package:=${DESCRIPTION_PKG} description_file:=${DESCRIPTION_FILE} launch_rviz:=false tool_voltage:=24"
```

# Run gripper service to enable the gripper 
```bash
podman run -it --network host --ipc=host --pid=host \
    ${GHCR_POINTER} \
    /bin/bash -c ". /root/ws/install/setup.sh && \
    ros2 run gripper_service gripper_service"
```

# Launch move_group
```bash
podman run -it --network host --ipc=host --pid=host \
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
