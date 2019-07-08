#!/bin/bash

################################################################################

# Link the default shell 'sh' to Bash.
alias sh='/bin/bash'

################################################################################

# TODO: Does this have any effect on the current shell?
# Source the ROS environment.
source /opt/ros/kinetic/setup.bash
source /root/catkin_ws/devel/setup.bash

################################################################################

# Define Bash functions to conveniently execute the helper scripts in the current shell process.

function o2as-initialize-catkin-workspace () {
  # Store the current directory and execute scripts in the current shell process.
  pushd .
  source /root/scripts/initialize-catkin-workspace.sh
  source /root/scripts/fix-permission-issues.sh
  popd
}

function o2as-reset-catkin-workspace () {
  # Store the current directory and execute scripts in the current shell process.
  pushd .
  source /root/scripts/reset-catkin-workspace.sh
  source /root/scripts/fix-permission-issues.sh
  popd
}

function o2as-fix-permission-issues () {
  # Store the current directory and execute scripts in the current shell process.
  pushd .
  source /root/scripts/fix-permission-issues.sh
  popd
}

function o2as-run-vscode-editor () {
  # Store the current directory and execute scripts in the current shell process.
  pushd .
  source /root/scripts/run-vscode-editor.sh
  popd
}

################################################################################

# Set ROS network interface.

export ROS_IP=127.0.0.1
# echo "ROS_IP is set to '${ROS_IP}'."
export ROS_HOME=~/.ros
