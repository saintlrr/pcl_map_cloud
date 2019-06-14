cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcl_viewer_pose)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE release)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBARARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(PCL 1.7 REQUIRED)
find_package(G2O REQUIRED)
IF(G2O_FOUND)
    include_directories(${G2O_INCLUDE_DIR})
    link_directories(${G2O_LIBARARY_DIRS})
    message("G2O lib is found:"${G2O_INCLUDE_DIR})
ENDIF(G2O_FOUND)
#SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator 
#    g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d 
#    g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only 
#    g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper 
#    g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg 
#    g2o_types_data g2o_types_sim3 cxsparse )

include_directories(${PCL_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcl_viewer_pose src/pcl_viewer_pose.cpp src/config.cpp)
target_link_libraries (pcl_viewer_pose ${PCL_LIBRARIES} ${G2O_LIBRARIES})

add_executable (pose_slam src/pose_slam.cpp src/config.cpp)
target_link_libraries (pose_slam ${PCL_LIBRARIES} ${G2O_LIBRARIES})


add_executable (voxel_grid src/voxel_grid.cpp)
target_link_libraries (voxel_grid ${PCL_LIBRARIES})

add_executable (csv_visualization src/csv_visualization.cpp)
target_link_libraries (csv_visualization ${PCL_LIBRARIES})