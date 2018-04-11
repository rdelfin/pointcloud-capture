cmake_minimum_required(VERSION 2.8)
project(kinect_pcl_bridge)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

add_executable(kinect_reader main.cpp net_client.cpp)
set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT "kinect_reader")

# Find Packages
find_package(PCL REQUIRED)
find_package(KinectSDK2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${KinectSDK2_INCLUDE_DIRS})

# Preprocessor Definitions
add_definitions(${PCL_DEFINITIONS})

# Additional Library Directories
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${KinectSDK2_LIBRARY_DIRS})

# Additional Dependencies
target_link_libraries(kinect_reader ${PCL_LIBRARIES})
target_link_libraries(kinect_reader ${KinectSDK2_LIBRARIES})
