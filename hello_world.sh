#!/bin/bash
source /opt/ros/galactic/setup.bash
source ~/Projects/Newcastle-Racing/install/setup.bash
sudo /home/fsai/FS-AI_API/setup.sh
ros2 launch newcastle_racing_ai static_integration.launch.py
# /home/fsai/FS-AI_API/FS-AI_API_Console/fs-ai_api_console can0
read done
