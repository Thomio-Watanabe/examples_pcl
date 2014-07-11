cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(3D_features_integral_images)
find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(integral_images integral_images.cpp)
target_link_libraries(integral_images ${PCL_LIBRARIES})

