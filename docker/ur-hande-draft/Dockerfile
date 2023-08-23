FROM ghcr.io/nsls2/erobs-ur-driver:latest

RUN apt-get update && apt-get install -y \
  build-essential \
  cmake \
  git \
  # ament-cmake \
  # ament-cmake-pytest \
  # ros-${ROS_DISTRO}-rosidl-default-generators \
  ros-dev-tools \
  ros-${ROS_DISTRO}-moveit \
  ros-${ROS_DISTRO}-moveit-visual-tools \
  ros-${ROS_DISTRO}-ros2-control \
  ros-${ROS_DISTRO}-ros2-controllers \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget

RUN apt-get update
RUN apt-get install -y lsb-release

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


RUN mkdir /root/ws
WORKDIR /root/ws

RUN git clone https://maffettone:${GITHUB_TOKEN}@github.com/ChandimaFernando/ERoBS_ur3e.git -b action_client_basic

WORKDIR /root/ws/ERoBS_ur3e/external_dependencies
RUN vcs import < ERoBS_dependencies.humble.repos
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash  && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y"

WORKDIR  /root/ws/ERoBS_ur3e/
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --packages-select custom_msgs"
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
  source /root/ws/ERoBS_ur3e/install/setup.bash && \
  colcon build --symlink-install --packages-select \
  warehouse_ros_mongo \
  moveit_task_constructor_core \
  moveit_task_constructor_msgs \
  ur3e_hande_description \
  ur_hande_moveit_config \
  serial \
  robotiq_driver \
  rviz_marker_tools \
  cpp_arm_moveit \
  sample_movement_ur3e_sim \
  # Ignore 
  --packages-ignore \
  control_msgs \
  controller_manager_msgs \
  launch_param_builder \
  moveit_common \
  moveit_visual_tools \
  realtime_tools \
  ros2_control_test_assets \
  srdfdom \
  hardware_interface \
  moveit_configs_utils \
  controller_interface \
  moveit_core \
  controller_manager \
  moveit_ros_occupancy_map_monitor \
  moveit_simple_controller_manager \
  moveit_ros_planning \
  moveit_kinematics \
  moveit_planners_ompl \
  moveit_ros_robot_interaction \
  moveit_ros_warehouse \
  moveit_ros_move_group \
  moveit_ros_planning_interface \
  moveit_ros_visualization \
  moveit_setup_framework \
  pilz_industrial_motion_planner \
  moveit_planners \
  moveit_setup_app_plugins \
  moveit_setup_controllers \
  moveit_setup_core_plugins \
  moveit_setup_srdf_plugins \
  moveit_setup_assistant"

# RUN echo "source /root/ws/ERoBS_ur3e/install/setup.sh" >> /root/.bashrc

# Clean up
RUN apt-get autoremove -y \
  && apt-get clean -y \
  && rm -rf /var/lib/apt/lists/*

ENV ROBOT_IP=192.168.56.101
ENV REVERSE_IP=192.168.56.102
ENV UR_TYPE="ur3e"

# Start a shell session when the container runs by default
CMD ["/bin/bash"]