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

RUN mkdir /root/ws/src -p

WORKDIR /root/ws/src

RUN git clone https://github.com/PickNikRobotics/serial-release.git -b release/humble/serial

WORKDIR /root/ws/

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
        colcon build --symlink-install --packages-select serial"

