#!/bin/bash

# The directory you want to change to
TARGET_DIR="/home/xplore/dev_ws/"

cd "$TARGET_DIR"
source /opt/ros/humble/setup.bash
source "install/setup.bash"
pip install evdev

ros2 launch hd_launch test.launch.py sim:=True vision:=False
echo "Finished running commands as root"