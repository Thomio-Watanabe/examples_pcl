cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(3D_features_extractor)
find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(3d_features 3d_features.cpp check_input.cpp)
target_link_libraries(3d_features ${PCL_LIBRARIES})

