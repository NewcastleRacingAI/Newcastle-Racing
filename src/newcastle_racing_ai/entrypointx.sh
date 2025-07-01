#!/bin/bash
source /opt/ros/galactic/setup.bash

# cd workspace/MPCC/C++
# rm -rf External/
# ./install.sh

# build path planning node
cd /workspace/ft-fsd-path-planning
pip install --no-cache-dir -r requirements.txt

cd /workspace
source /opt/ros/galactic/setup.bash
pip3 install --no-cache-dir pandas matplotlib scipy
rosdep install --from-paths . --ignore-src -r -y --skip-keys="mpcc_control"

#build ROS_CAN_API 
cd /workspace/ros_can/eufs_msgs
make
cd /workspace/ros_can/FS-AI_API/FS-AI_API
make
cd /workspace/ros_can/FS-AI_API
./setup.sh


# colcon build --packages-select newcastle_racing_ai newcastle_racing_ai_msgs
# colcon build --symlink-install
colcon build --symlink-install --packages-skip mpcc_control
source install/setup.bash

# launch the nodes
ros2 launch newcastle_racing_ai all_nodes.launch.py