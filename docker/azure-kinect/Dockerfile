FROM osrf/ros:humble-desktop

RUN apt-get update \
        && apt-get -y install \
        python3-pip \
        build-essential \
        cmake \
        usbutils \
        libgtk2.0-dev \
        libusb-1.0\
        ffmpeg \
        mlocate \
        wget \
        curl \
        ros-${ROS_DISTRO}-joint-state-publisher \
        build-essential \
        python3-colcon-common-extensions \
        software-properties-common

# For vnc viewer
RUN apt-get install -y x11vnc xvfb 

# Clean up
RUN apt-get autoremove -y \
        && apt-get clean -y \
        && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo tee /etc/apt/trusted.gpg.d/microsoft.asc
RUN sudo apt-add-repository https://packages.microsoft.com/ubuntu/20.04/prod

RUN curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
RUN sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod
RUN curl -sSL https://packages.microsoft.com/config/ubuntu/18.04/prod.list | sudo tee /etc/apt/sources.list.d/microsoft-prod.list
RUN curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -

# || true forces through the errors without breaking the apt-get update
RUN sudo apt-get update || true 

# End user license agreement (EULA) parameter is necessary. -y won't trigger an EULA answer
RUN ACCEPT_EULA=Y apt-get install -y libk4a1.4 libk4a1.4-dev

RUN wget mirrors.kernel.org/ubuntu/pool/universe/libs/libsoundio/libsoundio1_1.1.0-1_amd64.deb
RUN sudo dpkg -i libsoundio1_1.1.0-1_amd64.deb
RUN sudo apt install -y k4a-tools

# Copy the rules
COPY 99-k4a.rules /etc/udev/rules.d/99-k4a.rules

RUN pip3 install xacro 

RUN mkdir /root/temp_code/
RUN mkdir /root/ws/src -p

WORKDIR /root/temp_code/
RUN git clone https://github.com/ChandimaFernando/erobs.git -b azure-kinect
RUN mv /root/temp_code/erobs/src/kinect_recorder /root/ws/src/

WORKDIR /root/ws/src
RUN git clone https://github.com/microsoft/Azure_Kinect_ROS_Driver.git -b humble

WORKDIR /root/ws/
RUN /bin/bash -c ". /opt/ros/humble/setup.bash && colcon build"

# Set display
ENV DISPLAY :1
