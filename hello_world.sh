#!/bin/bash
######################################################################################
# Entry point script for Newcastle Racing AI system.
# It will be launched automatically by the real car once the system has fully loaded.
######################################################################################

# Source the ROS2 and workspace setup files
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

# Launch the necessary components
# sudo /home/fsai/FS-AI_API/setup.sh
# ros2 launch newcastle_racing_ai static_integration.launch.py
# /home/fsai/FS-AI_API/FS-AI_API_Console/fs-ai_api_console can0
/workspace/launch_simulator.sh
read done #this (i think) is to hang the process and prevent the container from exiting?
