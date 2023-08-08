FROM osrf/ros:humble-desktop-full


RUN apt-get update \
        && apt-get -y install \
        python3-pip \
        ros-${ROS_DISTRO}-rclpy \
        ros-${ROS_DISTRO}-ur \
        build-essential \
        python3-colcon-common-extensions \
        #
        # Clean up
        && apt-get autoremove -y \
        && apt-get clean -y \
        && rm -rf /var/lib/apt/lists/*
RUN pip3 install rosdep && \
        rosdep update

CMD ["/bin/sh", "-c", ".  /opt/ros/${ROS_DISTRO}/setup.sh && ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py"]
