cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(kinect_features)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIR})
add_definitions(${PCL_DEFINITIONS})

add_executable(kinect_features kinect_features.cpp)
target_link_libraries(kinect_features ${PCL_LIBRARIES})

