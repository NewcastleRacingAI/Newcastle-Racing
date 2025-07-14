#!/bin/bash
set -e

cd MPCC/C++
if [ ! -d "External" ]; then
    ./install.sh
else
    echo "External/ already exists — skipping install."
fi
cd /src/ft-fsd-path-planning
pip install --no-cache-dir -r requirements.txt
pip install cvxpy

# install ros_can
# cd /workspace/ros_can/FS-AI-API
# ./setup.sh

# # Build ROS packages

cd /src
source /opt/ros/galactic/setup.bash
#rosdep install --from-paths . --ignore-src -r -y --skip-keys "pandas,matplotlib,scipy"

# # ignore errors about pandas, matplolib and scipy

# build ros workspace
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-skip mpcc_control

# Source ROS and workspace
source /opt/ros/galactic/setup.bash
source ~/install/setup.bash


# launch the nodes
exec ros2 launch eufs_launcher eufs_launcher.launch.py
tail -f /dev/null
