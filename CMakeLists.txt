cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcl_viewer_pose)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE release)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcl_viewer_pose pcl_viewer_pose.cpp)
target_link_libraries (pcl_viewer_pose ${PCL_LIBRARIES})

add_executable (voxel_grid voxel_grid.cpp)
target_link_libraries (voxel_grid ${PCL_LIBRARIES})