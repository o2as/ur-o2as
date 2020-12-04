#!/bin/bash

################################################################################

# Initialize the Catkin workspace.
cd /root/catkin_ws/ && catkin_init_workspace

# Build the Catkin workspace.
cd /root/catkin_ws/ && catkin build

# TODO: This has no effect on the current shell.
# Source the Catkin workspace.
source /root/catkin_ws/devel/setup.bash

################################################################################

# Generate the URDF files in o2as_parts_description

rosrun o2as_parts_description generate_urdf_from_meshes.py
rosrun o2as_parts_description generate_assembled_piece.py
