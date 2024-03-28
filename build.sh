#!/bin/bash
set -e

# Set the default build type
BUILD_TYPE=RelWithDebInfo

source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build \
        --merge-install \
        --symlink-install \
        --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
        --packages-select ur3e_hande_robot_description serial robotiq_driver pdf_beamtime_interfaces \
        -Wall -Wextra -Wpedantic

colcon build \
        --merge-install \
        --symlink-install \
        --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
        -Wall -Wextra -Wpedantic