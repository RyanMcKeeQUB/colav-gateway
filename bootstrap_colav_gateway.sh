#!/bin/bash

# source ros2
source /opt/ros/humble/setup.bash

# Navigate to the ROS2 workspace
cd /home/ros2_ws 

# Build the specified packages
colcon build

# Source the ROS2 setup file
source /home/ros2_ws/install/setup.bash

# Launch the colav_gateway package
ros2 launch colav_gateway colav_gateway.launch.py

# Execute the container's default process
exec "$@"  # This will start the container's default process
