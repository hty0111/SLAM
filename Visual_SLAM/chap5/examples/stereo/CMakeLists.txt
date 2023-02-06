cmake_minimum_required(VERSION 3.0)
project(Project)

set(CMAKE_BUILD_TYPE Release)
ADD_COMPILE_OPTIONS(-std=c++11)
ADD_COMPILE_OPTIONS(-std=c++14)
set(CMAKE_CXX_FLAGS "-std=c++11 -O3")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

add_executable(stereo stereo.cpp)

target_link_libraries(
        stereo
        ${OpenCV_LIBS}
        ${Pangolin_LIBRARIES}
)