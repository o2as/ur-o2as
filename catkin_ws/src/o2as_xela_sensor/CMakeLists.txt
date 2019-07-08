cmake_minimum_required(VERSION 2.8.3)
project(o2as_xela_sensor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib_msgs
  message_generation
)

catkin_python_setup()

add_message_files(DIRECTORY msg)
add_service_files(DIRECTORY srv)
add_action_files(DIRECTORY action)
generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

catkin_package()
