cmake_minimum_required(VERSION 3.0)
project(loop_closure)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")

if (POLICY CMP0072)
    set(OpenGL_GL_PREFERENCE LEGACY)
endif()

find_package(g2o REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(OpenCV REQUIRED)
find_package(DBoW3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR} ${DBoW3_INCLUDE_DIRS})

add_executable(feature_training feature_training.cpp)
target_link_libraries(feature_training ${OpenCV_LIBS} ${DBoW3_LIBS})

add_executable(loop_closure loop_closure.cpp)
target_link_libraries(loop_closure ${OpenCV_LIBS} ${DBoW3_LIBS})

