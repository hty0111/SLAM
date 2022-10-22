cmake_minimum_required(VERSION 2.8)
project(Project)

# 添加Eigen头文件

#include_directories( "/usr/include/eigen3" )

find_package(Eigen3)
find_package(Pangolin REQUIRED)
include_directories(
        ${EIGEN3_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
)

add_executable(eigenMatrix eigenMatrix.cpp)

add_executable(eigenGeometry eigenGeometry.cpp)

add_executable(plotTrajectory plotTrajectory.cpp)
target_link_libraries(plotTrajectory ${Pangolin_LIBRARIES})