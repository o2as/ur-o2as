cmake_minimum_required(VERSION 2.8.3)
project(omron_cad_matching)

add_compile_options(-std=gnu++11 -O2 -fopenmp)

##############
## Packages ##
##############

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    rospy 
    dynamic_reconfigure
    std_msgs 
    sensor_msgs
    geometry_msgs 
    message_generation
    message_filters
    image_transport
    cv_bridge
    pcl_ros
    pcl_conversions 
)

find_package(OpenCV 3 REQUIRED COMPONENTS opencv_core opencv_highgui)

# OpenMP
find_package(OpenMP)
if(NOT OpenMP_FOUND)
    message(FATAL_ERROR "\n\n OpenMP is missing!\n\n")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -fopenmp")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

catkin_python_setup()
generate_dynamic_reconfigure_options(cfg/Search.cfg)

#########################
## message and service ##
#########################

add_message_files(DIRECTORY msg)
add_service_files(DIRECTORY srv)
generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    CATKIN_DEPENDS roscpp std_msgs geometry_msgs
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

set(cad_LIBRARIES
    ${PROJECT_SOURCE_DIR}/lib/libcommon.so
    #${PROJECT_SOURCE_DIR}/lib/avx/libcad.so
    ${PROJECT_SOURCE_DIR}/lib/sse/libcad.so
    #${PROJECT_SOURCE_DIR}/lib/avx/libcad_free.so
)

#=========================================
# Common 
#=========================================

# util_cam
add_library(${PROJECT_NAME}_util_cam src/util_cam.cpp)
target_link_libraries(${PROJECT_NAME}_util_cam ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

# util
add_library(${PROJECT_NAME}_util src/util.cpp)
target_link_libraries(${PROJECT_NAME}_util ${PROJECT_NAME}_util_cam ${cad_LIBRARIES})

#=========================================
# Stand alone (single model)
#=========================================

# train
add_executable(train src/train.cpp)
target_link_libraries(train ${PROJECT_NAME}_util)

# search
add_executable(search src/search.cpp)
target_link_libraries(search ${PROJECT_NAME}_util)

#=========================================
# Server
#=========================================

# train_server
add_executable(train_server src/train_server.cpp)
add_dependencies(train_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(train_server ${PROJECT_NAME}_util)

# search_server
add_executable(search_server src/search_server.cpp)
add_dependencies(search_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(search_server ${PROJECT_NAME}_util)

# data_collection_server
add_executable(data_collection_server src/data_collection_server.cpp)
add_dependencies(data_collection_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(data_collection_server ${PROJECT_NAME}_util_cam)
