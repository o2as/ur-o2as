cmake_minimum_required(VERSION 2.8.12)
project(o2as_phoxi_camera)

find_package(PhoXi REQUIRED CONFIG PATHS "$ENV{PHOXI_CONTROL_PATH}")

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    roslib
    rospy
    std_msgs
    sensor_msgs
    dynamic_reconfigure
    tf
)

find_package(OpenCV REQUIRED)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PHOXI_INCLUDE_DIRS}
)

add_message_files(
  FILES
    PhoXiSize.msg
)

add_service_files(
  FILES
    SetString.srv
    GetString.srv
    GetStringList.srv
    GetInt.srv
    GetSupportedCapturingModes.srv
    GetFrame.srv
    DumpFrame.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
    sensor_msgs
)

generate_dynamic_reconfigure_options(
  cfg/o2as_phoxi_camera.cfg
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    message_runtime
    std_msgs
    sensor_msgs 
)

add_compile_options(-w)
add_compile_options(-std=c++14)
add_compile_options(-fpermissive)
add_compile_options(-pthread)

add_executable(
  ${PROJECT_NAME}
  src/main.cpp
)

add_dependencies(
  ${PROJECT_NAME}
  ${PROJECT_NAME}_gencfgv
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ${PROJECT_NAME}
  debug ${catkin_LIBRARIES}
  optimized ${catkin_LIBRARIES}
  ${PHOXI_LIBRARY}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  rt
  yaml-cpp
)

install(
  PROGRAMS
    src/o2as_phoxi_camera_example.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES
    ${PHOXI_LIB_RELEASE_PER_COMPILER}.${PHO_SOFTWARE_VERSION}
  DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)
