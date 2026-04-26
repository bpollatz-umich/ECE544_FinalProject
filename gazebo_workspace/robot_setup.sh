#!/bin/bash
source /opt/ros/jazzy/setup.bash

# Ensure GUI variables are correctly pointed
export DISPLAY=:0
export LD_LIBRARY_PATH=/usr/lib/wsl/lib:$LD_LIBRARY_PATH
export LIBGL_ALWAYS_SOFTWARE=1 

echo "GUI and ROS 2 Environment Ready!"
