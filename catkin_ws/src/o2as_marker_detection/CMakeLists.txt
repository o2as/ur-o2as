cmake_minimum_required(VERSION 2.8.3)
project(o2as_marker_detection)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  rospy
  roscpp
  dynamic_reconfigure
  std_srvs
  geometry_msgs
)

catkin_python_setup()
generate_dynamic_reconfigure_options(
  cfg/MarkerDetection.cfg
)
catkin_package()
