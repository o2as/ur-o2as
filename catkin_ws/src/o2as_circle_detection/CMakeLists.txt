################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(o2as_circle_detection)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  sensor_msgs
  cv_bridge
)

################################################################################
# Declare ROS messages, services and actions
################################################################################

# add_service_files(
#   DIRECTORY
#   srv
# )

# add_action_files(DIRECTORY action )

# generate_messages(
#   DEPENDENCIES actionlib_msgs std_msgs control_msgs
# )

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  # INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# add_executable(ServiceClient_test src/ServiceClient_test.cpp)
# add_dependencies(ServiceClient_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(ServiceClient_test ${catkin_LIBRARIES})
