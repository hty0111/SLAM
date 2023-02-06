cmake_minimum_required(VERSION 3.0)
project(Project)

set(CMAKE_BUILD_TYPE Release)
ADD_COMPILE_OPTIONS(-std=c++11)
ADD_COMPILE_OPTIONS(-std=c++14)
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")


if (POLICY CMP0072)
    set(OpenGL_GL_PREFERENCE LEGACY)
endif()


find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(g2o REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(gaussNewton gaussNewton.cpp)
add_executable(ceresFitting ceresFitting.cpp)
add_executable(g2oFitting g2oFitting.cpp)

target_link_libraries(gaussNewton ${OpenCV_LIBS})
target_link_libraries(ceresFitting ${OpenCV_LIBS} ${CERES_LIBRARIES})
target_link_libraries(g2oFitting ${OpenCV_LIBS} g2o_core g2o_stuff)
