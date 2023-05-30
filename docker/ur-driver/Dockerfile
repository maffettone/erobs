FROM althack/ros2:humble-base

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
        && apt-get -y install --no-install-recommends \
        ros-${ROS_DISTRO}-rclpy \
        ros-${ROS_DISTRO}-ur-robot-driver \
        #
        # Clean up
        && apt-get autoremove -y \
        && apt-get clean -y \
        && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

RUN . /opt/ros/${ROS_DISTRO}/setup.sh

ENV ROBOT_IP=192.168.56.101
CMD ["/bin/bash", "-c", ".  /opt/ros/${ROS_DISTRO}/setup.sh && ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=${ROBOT_IP} launch_rviz:=false"]
