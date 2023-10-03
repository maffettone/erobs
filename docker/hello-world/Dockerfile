# Minimal source install to run basic tests and communication debugging.
FROM osrf/ros:humble-desktop-full
RUN apt-get update && apt-get install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget \
  ros-${ROS_DISTRO}-ur \
  ros-${ROS_DISTRO}-rclpy \
  python3-colcon-common-extensions \
  ros-dev-tools

# Emprical best practice is to do all ros dependencies packages first!
RUN pip3 install numpy rosdep && \
  rosdep update

ARG GITHUB_TOKEN

ARG CACHE_BUST

RUN mkdir /root/ws/src -p
WORKDIR  /root/ws/src
RUN git clone https://github.com/ChandimaFernando/erobs.git -b humble
#  https://maffettone:${GITHUB_TOKEN}@github.com/maffettone/erobs.git -b humble
WORKDIR  /root/ws/
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash  && \
  rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y"

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
  colcon build --symlink-install --packages-select  hello_moveit_interfaces"

RUN /bin/bash -c "source install/setup.bash && \
  colcon build --symlink-install"


# This is an exit hatch for testing when conda envs not avail
# RUN pip3 install\
#   bluesky \
#   ophyd \
#   nslsii \
#   ipython

# Clean up
RUN apt-get autoremove -y \
  && apt-get clean -y \
  && rm -rf /var/lib/apt/lists/*

CMD ["/bin/bash"]

