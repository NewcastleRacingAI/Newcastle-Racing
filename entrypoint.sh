#!/bin/bash
set -e

# Source ROS and workspace
source /opt/ros/galactic/setup.bash
source /workspace/install/setup.bash

exec "$@"

