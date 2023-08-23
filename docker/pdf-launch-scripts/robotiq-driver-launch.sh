#!/bin/bash
docker run -it --network host\
    ghcr.io/nsls2/ur-hande-draft:latest \
    /bin/bash -c ". /root/ws/ERoBS_ur3e/install/setup.sh && \
ros2 launch robotiq_driver tests.launch.py"
