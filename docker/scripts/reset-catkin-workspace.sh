#!/bin/bash

################################################################################

# Download package lists from Ubuntu repositories.
apt-get update

# Install system dependencies required by specific ROS packages.
# http://wiki.ros.org/rosdep
rosdep update

# TODO: Does this have any effect on the current shell?
# Source the ROS environment.
source /opt/ros/kinetic/setup.bash

################################################################################

# Remove the Catkin workspace.
cd /root/catkin_ws/ && catkin clean -y
cd /root/catkin_ws/ && rm -r CMakeLists.txt .catkin_tools/

################################################################################

# Initialize the Catkin workspace.
cd /root/catkin_ws/ && catkin_init_workspace

# Build the Catkin workspace.
cd /root/catkin_ws/ && catkin build

# TODO: This has no effect on the current shell.
# Source the Catkin workspace.
source /root/catkin_ws/devel/setup.bash
