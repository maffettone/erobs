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

RUN git clone https://github.com/ChandimaFernando/erobs.git -b humble_hande_integration

WORKDIR /root/ws/

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
        colcon build --symlink-install --packages-select ur3e_hande_robot_description"

ENV ROBOT_IP=192.168.56.101
ENV REVERSE_IP=192.168.56.102
ENV UR_TYPE="ur3e"
ENV DESCRIPTION_PKG="ur3e_hande_robot_description"
ENV DESCRIPTION_FILE="ur_with_hande.xacro"

# Copy entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]
