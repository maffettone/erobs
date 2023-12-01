# Minimal working example for running BSUI on a beamline srv
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
  ros-${ROS_DISTRO}-rclpy \
  python3-colcon-common-extensions \
  ros-dev-tools

# Emprical best practice is to do all ros dependencies packages first!
RUN pip3 install numpy rosdep && \
  rosdep update

ARG GITHUB_TOKEN
RUN mkdir /root/ws/src -p
WORKDIR  /root/ws/src
RUN git clone https://maffettone:${GITHUB_TOKEN}@github.com/maffettone/erobs.git -b humble
WORKDIR  /root/ws/
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash  && \
  rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y"
WORKDIR  /root/ws/src
RUN git clone https://maffettone:${GITHUB_TOKEN}@github.com/ChandimaFernando/ERoBS_ur3e.git -b action_client_basic
WORKDIR  /root/ws/
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
  colcon build --packages-select hello_moveit_interfaces custom_msgs"


# This is an exit hatch for testing when conda envs not avail
RUN pip3 install\
  bluesky \
  ophyd \
  nslsii \
  ipython

RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O miniconda.sh && \
  chmod +x miniconda.sh && \
  ./miniconda.sh -b -p /opt/conda && \
  rm miniconda.sh

ENV PATH="/opt/conda/bin:$PATH"

# Install Epics
RUN mkdir $HOME/EPICS && cd $HOME/EPICS && \
  git clone --recursive https://github.com/epics-base/epics-base.git && \
  cd epics-base && \
  make

ENV EPICS_BASE=/root/EPICS/epics-base
# ENV EPICS_HOST_ARCH=$(/root/EPICS/epics-base/startup/EpicsHostArch)
# Set host arch manually
ENV PATH=${EPICS_BASE}/bin/linux-x86_64:${PATH} 

# Add our custom bsui
COPY bsui.bash /bin/bsui
RUN chmod +x /bin/bsui

RUN apt-get update && apt-get install -y fontconfig 
RUN mkdir /root/.ipython

# Clean up
# TODO (maffettone): this image is rather large, there may be some source and build files we can clean up here. 
RUN apt-get autoremove -y \
  && apt-get clean -y \
  && rm -rf /var/lib/apt/lists/*

# Bluesky environment variables
ENV BS_ENV="/nsls2/conda/envs/2023-2.1-py310-tiled"
ENV BS_PROFILE="collection"

# Start a shell session with BSUI when we run the container
CMD ["/bin/bash", "-c", ".  /opt/ros/${ROS_DISTRO}/setup.bash &&  . /root/ws/install/setup.bash && bsui"]
