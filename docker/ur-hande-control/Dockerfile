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
RUN git clone https://github.com/ChandimaFernando/erobs.git -b humble_hande_control

WORKDIR /root/ws/

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
        colcon build --symlink-install --packages-select pdf_beamtime_interfaces serial robotiq_driver"

CMD ["/bin/sh", "-c", ".  /opt/ros/${ROS_DISTRO}/setup.sh && .  /root/ws/install/setup.sh && ros2 run robotiq_driver gripper_service"]
