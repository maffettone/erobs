FROM althack/ros2:humble-dev

ENV OVERLAY_WS=/opt/overlay_ws
ARG CONTROL_BRANCH=2.24.1

ENV DEBIAN_FRONTEND=noninteractive

# Added a few dependencies needed to source install ros2_control as rosdep install failed on those dependencies
RUN apt-get update \
        && apt-get -y install --no-install-recommends \
        ros-${ROS_DISTRO}-rclpy \
        ros-${ROS_DISTRO}-ur-robot-driver \
        ros-${ROS_DISTRO}-diagnostic-updater \
        ros-${ROS_DISTRO}-ros2-controllers \
        graphviz graphviz-dev \
        ros-${ROS_DISTRO}-rqt-gui \
        ros-${ROS_DISTRO}-rqt-gui-py \       
        git \
        build-essential \
        python3-colcon-common-extensions \ 
        && pip3 install pygraphviz


RUN pip3 install rosdep && \
        rosdep update

RUN git clone --depth 1 --branch $CONTROL_BRANCH https://github.com/ros-controls/ros2_control.git $OVERLAY_WS/src/
WORKDIR $OVERLAY_W/src

## Redundant bacause of the previous git clone
# ADD https://raw.githubusercontent.com/ros-controls/ros2_control/$CONTROL_BRANCH/ros2_control.$ROS_DISTRO.repos ros2_control.repos
# RUN vcs import < ros2_control.repos
# RUN apt-get update && rosdep update && \
#         rosdep install -iy --from-paths . && \
#         rm -rf /var/lib/apt/lists/

WORKDIR $OVERLAY_WS
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
        colcon build

ENV DEBIAN_FRONTEND=dialog

ENV ROBOT_IP=192.168.56.101
CMD ["/bin/sh", "-c", ".  /opt/ros/${ROS_DISTRO}/setup.sh && . ${OVERLAY_WS}/install/setup.sh && ros2 pkg xml ros2_control | grep version && ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=${ROBOT_IP} launch_rviz:=false"]
# Needed to source the workspace at the runtime to include the source installation of ros2_control
# ros2 pkg xml ros2_control | grep version  :  verifies if we have the correct control branch installed