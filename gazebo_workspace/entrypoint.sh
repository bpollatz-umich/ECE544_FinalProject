#!/bin/bash
set -e

# 1. Source ROS Jazzy for the current session
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
fi

# 2. Permanent Fix: Add to .bashrc for future 'docker exec' tabs
# We check if it's already there first to avoid duplicate lines
if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
fi

# 3. Graphics Setup
export DISPLAY=:0
export LIBGL_ALWAYS_SOFTWARE=1

exec "$@"
