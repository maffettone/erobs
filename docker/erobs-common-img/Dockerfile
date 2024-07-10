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

RUN mkdir /root/ws -p

WORKDIR /root/ws

# RUN git clone https://github.com/maffettone/erobs.git -b humble
RUN git clone https://github.com/ChandimaFernando/erobs.git -b humble_fiducial_pose
# RUN git clone https://github.com/ChandimaFernando/erobs.git -b fix_hande_urdf \ 
# && cd erobs \
# && git checkout a31fe9852ca8803aec94615fa80d6a402869a210

WORKDIR /root/ws/erobs/
RUN /bin/bash -c ". ./setup.sh"

WORKDIR /root/ws/
RUN /bin/bash -c ". ./erobs/build.sh"

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
