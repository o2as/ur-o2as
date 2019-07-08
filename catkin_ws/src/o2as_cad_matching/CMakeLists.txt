cmake_minimum_required(VERSION 2.8.3)
project(o2as_cad_matching)

add_compile_options(-std=gnu++11 -O2 -fopenmp)

##############
## Packages ##
##############

find_package(catkin REQUIRED COMPONENTS 
    rospy 
    roscpp 
    dynamic_reconfigure
    std_msgs 
    sensor_msgs
    geometry_msgs 
    message_generation
    message_filters
    image_transport
    cv_bridge
    pcl_conversions
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

# OpenMP
find_package(OpenMP)
if(NOT OpenMP_FOUND)
    message(FATAL_ERROR "\n\n OpenMP is missing!\n\n")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -fopenmp")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

catkin_python_setup()
generate_dynamic_reconfigure_options(cfg/CadMatching.cfg)

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
