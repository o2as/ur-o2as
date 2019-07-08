cmake_minimum_required(VERSION 2.8.3)
project(o2as_vision)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  rospy
  roscpp
  std_srvs
  o2as_phoxi_camera
  o2as_realsense_camera
  shape_msgs
  geometry_msgs
  geometric_shapes
  o2as_cad_matching
)

# find_package(Boost REQUIRED COMPONENTS system)
catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

catkin_package(
)

###########
## Build ##
###########

catkin_install_python(PROGRAMS scripts/phoxi_camera_actual_example.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
#install(
#  PROGRAMS
#    scripts/cad_matching_interface.py
#    scripts/planning_scene_interface.py
#    scripts/realsense_camera_interface.py
#    scripts/vision_group_interface.py
#    scripts/vision_group.py
#  DESTINATION
#    ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_o2as_vision.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
