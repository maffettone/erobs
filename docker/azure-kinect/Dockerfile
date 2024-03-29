FROM osrf/ros:humble-desktop-full

# Set display
ENV DISPLAY :1

RUN apt-get update \
        && apt-get -y install \
        python3-pip \
        build-essential \
        cmake \
        libgtk2.0-dev \
        libusb-1.0\
        ffmpeg \
        mlocate \
        wget \
        curl \
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

# End user license agreement (EULA) parameter is necessary. -y won't trigger a EULA answer
RUN ACCEPT_EULA=Y apt-get install -y libk4a1.4 libk4a1.4-dev

RUN wget mirrors.kernel.org/ubuntu/pool/universe/libs/libsoundio/libsoundio1_1.1.0-1_amd64.deb
RUN sudo dpkg -i libsoundio1_1.1.0-1_amd64.deb
RUN sudo apt install -y k4a-tools

# RUN sudo k4arecorder

# VNC viewer
EXPOSE 5905

CMD ["/bin/bash"]
# # Copy entrypoint script
# COPY entrypoint.sh /entrypoint.sh
# RUN chmod +x /entrypoint.sh
# ENTRYPOINT [ "/bin/bash", "/entrypoint.sh" ]
