FROM dog_waste_retrieval:latest

# Install Gazebo and the ROS bridge
RUN apt-get update && apt-get install -y \
    ros-humble-ros-gz \
    && rm -rf /var/lib/apt/lists/*

# Source ROS automatically when you open a terminal
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
