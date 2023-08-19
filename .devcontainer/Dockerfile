FROM althack/ros2:humble-dev 

# ** [Optional] Uncomment this section to install additional packages. **
#
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
        && apt-get -y install --no-install-recommends \
        python3-pip \
        ros-${ROS_DISTRO}-rclpy \
        ros-${ROS_DISTRO}-ur \
        ros-${ROS_DISTRO}-moveit\
        build-essential \
        python3-colcon-common-extensions \
        #
        # Clean up
        && apt-get autoremove -y \
        && apt-get clean -y \
        && rm -rf /var/lib/apt/lists/*

RUN pip3 install bluesky ophyd Ipython rosidl_adapter
ENV DEBIAN_FRONTEND=dialog

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc
