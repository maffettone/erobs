FROM althack/ros2:humble-dev

ARG OVERLAY_WS=/opt/overlay_ws
ARG CONTROL_BRANCH=2.24.1

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
        && apt-get -y install --no-install-recommends \
        ros-${ROS_DISTRO}-rclpy \
        ros-${ROS_DISTRO}-ur-robot-driver \
        ros-${ROS_DISTRO}-diagnostic-updater \
        git \
        build-essential \
        python3-colcon-common-extensions \ 
        && apt-get remove ros-humble-ros2-control \
        # Clean up
        && apt-get autoremove -y \
        && apt-get clean -y \
        && rm -rf /var/lib/apt/lists/*

RUN pip3 install rosdep && \
        rosdep update

RUN git clone --depth 1 --branch $CONTROL_BRANCH https://github.com/ros-controls/ros2_control.git $OVERLAY_WS/src/
WORKDIR $OVERLAY_W/src
ADD https://raw.githubusercontent.com/ros-controls/ros2_control/$CONTROL_BRANCH/ros2_control.$ROS_DISTRO.repos ros2_control.repos
RUN vcs import < ros2_control.repos
RUN apt-get update && rosdep update && \
        rosdep install -iy --from-paths . && \
        rm -rf /var/lib/apt/lists/

WORKDIR $OVERLAY_WS
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
        colcon build

ENV DEBIAN_FRONTEND=dialog

ENV ROBOT_IP=192.168.56.101
CMD ["/bin/bash", "-c", ".  /opt/ros/${ROS_DISTRO}/setup.sh && ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=${ROBOT_IP} launch_rviz:=false"]
