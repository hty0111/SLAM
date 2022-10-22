cmake_minimum_required(VERSION 2.8)
project(Project)

find_package(Eigen3)
find_package(Pangolin REQUIRED)
include_directories(
        ${EIGEN3_INCLUDE_DIRS}
)

add_executable(5 5.cpp)
add_executable(6 6.cpp)

