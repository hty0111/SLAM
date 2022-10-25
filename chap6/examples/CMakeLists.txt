cmake_minimum_required(VERSION 3.0)
project(Project)

set(CMAKE_BUILD_TYPE Release)
ADD_COMPILE_OPTIONS(-std=c++11)
ADD_COMPILE_OPTIONS(-std=c++14)
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(Ceres REQUIRED)

include_directories(
        ${EIGEN3_INCLUDE_DIRS}
#        ${Ceres_INCLUDE_DIRS}
)

add_executable(gaussNewton gaussNewton.cpp)
#add_executable(ceresFitting ceresFitting.cpp)

target_link_libraries(gaussNewton ${OpenCV_LIBS}})
#target_link_libraries(ceresFitting ${OpenCV_LIBS}} ${CERES_LIBRARIES})