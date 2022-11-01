cmake_minimum_required(VERSION 3.0)
project(Project)

set(CMAKE_BUILD_TYPE Release)
ADD_COMPILE_OPTIONS(-std=c++11)
ADD_COMPILE_OPTIONS(-std=c++14)
ADD_COMPILE_OPTIONS(-std=c++17)
set(CMAKE_CXX_FLAGS "-std=c++17 -O3")


if (POLICY CMP0072)
    set(OpenGL_GL_PREFERENCE LEGACY)
endif()


find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(g2o REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(5_1 5_1.cpp)
add_executable(5_2 5_2.cpp)

target_link_libraries(5_1 ${CERES_LIBRARIES})
target_link_libraries(5_2 ${CERES_LIBRARIES})
