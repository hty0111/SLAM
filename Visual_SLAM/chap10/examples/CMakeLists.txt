cmake_minimum_required(VERSION 3.0)
project(pose_graph)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17 -O3")

if (POLICY CMP0072)
    set(OpenGL_GL_PREFERENCE LEGACY)
endif()

find_package(g2o REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(pose_graph_SE3 pose_graph_SE3.cpp)
target_link_libraries(pose_graph_SE3 g2o_core g2o_stuff g2o_types_slam3d)

add_executable(pose_graph_se3 pose_graph_se3.cpp)
target_link_libraries(pose_graph_se3 g2o_core g2o_stuff fmt ${Sophus_LIBRARIES})
