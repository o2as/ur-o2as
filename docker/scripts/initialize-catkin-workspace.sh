#!/bin/bash

################################################################################

# Initialize the Catkin workspace.
cd /root/catkin_ws/ && catkin_init_workspace

# Build the Catkin workspace.
cd /root/catkin_ws/ && catkin build

# TODO: This has no effect on the current shell.
# Source the Catkin workspace.
source /root/catkin_ws/devel/setup.bash
