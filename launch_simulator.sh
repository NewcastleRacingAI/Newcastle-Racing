#!/bin/bash
######################################################################################
# Entry point script for the Docker container running the EUFS simulator.
######################################################################################
echo "Newcastle Racing AI System is starting..."
echo "Waiting for 5 seconds to ensure all services are up..."
sleep 5
echo "Starting ROS2 components..."
# Source the ROS2 and workspace setup files
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch the eufs launcher (starts the simulator)
ros2 launch eufs_launcher eufs_launcher.launch.py
