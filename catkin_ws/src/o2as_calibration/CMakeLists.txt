cmake_minimum_required(VERSION 2.8.3)
project(o2as_calibration)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  rospy
  roscpp
  std_srvs
  geometry_msgs
)

catkin_python_setup()
catkin_package()
