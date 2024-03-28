FROM osrf/ros:humble-desktop-full

# Set display
ENV DISPLAY :1

RUN apt-get update \
        && apt-get -y install \
        python3-pip \
        ros-${ROS_DISTRO}-rclpy \
        ros-${ROS_DISTRO}-ur \
        build-essential \
        python3-colcon-common-extensions
RUN apt-get install -y x11vnc xvfb 
# Clean up
RUN apt-get autoremove -y \
        && apt-get clean -y \
        && rm -rf /var/lib/apt/lists/*
RUN pip3 install rosdep && \
        rosdep update

RUN mkdir /root/ws/src -p

WORKDIR /root/ws/src

# TODO @ChandimaFernando. Revert back after tests. RUN git clone https://github.com/maffettone/erobs.git -b humble
RUN git clone https://github.com/ChandimaFernando/erobs.git -b humble_hande_control

WORKDIR /root/ws/src/erobs
RUN /bin/bash -c ". /root/ws/src/erobs/setup.sh"

# WORKDIR /root/ws/src/
WORKDIR /root/ws/src/erobs/

RUN /bin/bash -c "ls /root/ws/src/erobs/"
RUN /bin/bash -c "ls /root/ws/src/erobs/src"

RUN /bin/bash -c "cat /root/ws/src/erobs/build.sh"
RUN /bin/bash -c ". /root/ws/src/erobs/build.sh"

# RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
#         colcon build --symlink-install --packages-select ur3e_hande_robot_description && \
#         colcon build --symlink-install --packages-select ur3e_hande_moveit_config"

# RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
#         colcon build --symlink-install --packages-select pdf_beamtime_interfaces serial robotiq_driver pdf_beamtime"

# RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
#         source /root/ws/install/setup.bash && \  
#         colcon build --symlink-install --packages-select gripper_service"

WORKDIR /root/ws/

ENV ROBOT_IP=192.168.56.101
ENV REVERSE_IP=192.168.56.102
ENV UR_TYPE="ur3e"
ENV LAUNCH_RVIZ="false"
ENV DESCRIPTION_PKG="ur3e_hande_robot_description"
ENV DESCRIPTION_FILE="ur_with_hande.xacro"
ENV CONFIG_PKG="ur3e_hande_moveit_config"
ENV CONFIG_FILE="ur.srdf"

# VNC viewer
EXPOSE 5901
