# Draft container for action server for first beamtime, to be deprecated for future beamtimes. 
FROM osrf/ros:humble-desktop-full

RUN apt-get update && apt-get install -y \
  build-essential \
  cmake \
  git \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget


RUN pip3 install rosdep && \
  rosdep update

RUN pip3 install\
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.sh" >> /root/.bashrc

ARG GITHUB_TOKEN


RUN mkdir /root/ws/src -p
WORKDIR /root/ws/src

RUN git clone https://maffettone:${GITHUB_TOKEN}@github.com/ChandimaFernando/ERoBS_ur3e.git -b action_client_basic

WORKDIR /root/ws/src/ERoBS_ur3e/external_dependencies
RUN vcs import < ERoBS_dependencies.humble.repos
WORKDIR /root/ws/
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash  && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y"

WORKDIR  /root/ws/
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --packages-select custom_msgs"
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
  source /root/ws/install/setup.bash && \
  colcon build --symlink-install --packages-ignore custom_msgs"

# Clean up
RUN apt-get autoremove -y \
  && apt-get clean -y \
  && rm -rf /var/lib/apt/lists/*

ENV ROBOT_IP=192.168.56.101
ENV REVERSE_IP=192.168.56.102
ENV UR_TYPE="ur3e"

# Start a shell session when the container runs by default
CMD ["/bin/bash"]
