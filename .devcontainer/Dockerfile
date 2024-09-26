FROM althack/ros2:humble-full

# ** [Optional] Uncomment this section to install additional packages. **
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
        && apt-get -y install --no-install-recommends \
        python3-pip \
        python3.10-venv \
        ros-${ROS_DISTRO}-rclpy \
        ros-${ROS_DISTRO}-ur \
        ros-${ROS_DISTRO}-moveit\
        build-essential \
        python3-colcon-common-extensions \
        libopencv-dev \
        python3-opencv \
        # Clean up
        && apt-get autoremove -y \
        && apt-get clean -y \
        && rm -rf /var/lib/apt/lists/*

# Use venv for bluesky and ipython. Will need to call `source /venv/bin/activate`
RUN python3 -m venv venv
RUN . venv/bin/activate && pip install bluesky ophyd Ipython redis
ENV DEBIAN_FRONTEND=dialog

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc