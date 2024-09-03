#!/bin/bash
set -e

# Source the ROS 2 environment
source /opt/ros/foxy/setup.bash
# source /ros2_ws/install/setup.bash

# Start the ROS 2 nodes or launch file
exec "$@"
