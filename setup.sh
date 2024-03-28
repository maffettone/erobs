#!/bin/bash
set -e

vcs import < /root/ws/src/erobs/src/ros2.repos /root/ws/src/erobs/src
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -y
