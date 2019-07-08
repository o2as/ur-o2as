cmake_minimum_required(VERSION 2.8.3)
project(o2as_realsense_camera)

#add_compile_options(-std=gnu++11 -O2 -fopenmp)
add_compile_options(-std=c++11 -O2 -fopenmp)

find_package(catkin REQUIRED COMPONENTS
    roscpp 
    nodelet
    pluginlib
    dynamic_reconfigure
    message_generation
    std_srvs
    cv_bridge
    sensor_msgs
    image_transport
    pcl_conversions
    tf2_ros
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV 3 REQUIRED COMPONENTS opencv_core opencv_highgui)
find_package(realsense2)
if(NOT realsense2_FOUND)
    message(FATAL_ERROR "\n\n Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases\n\n")
endif()

catkin_python_setup()
generate_dynamic_reconfigure_options(cfg/RealSenseCamera.cfg)

add_service_files(DIRECTORY srv)
generate_messages(DEPENDENCIES sensor_msgs)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
)

add_library(realsense_camera src/RealSenseCamera.cpp)
target_link_libraries(realsense_camera realsense2 ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(realsense_camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(o2as_realsense_camera_nodelet src/nodelet.cpp)
target_link_libraries(o2as_realsense_camera_nodelet realsense_camera ${catkin_LIBRARIES})

install(TARGETS o2as_realsense_camera_nodelet LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(FILES o2as_realsense_camera_nodelet.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
