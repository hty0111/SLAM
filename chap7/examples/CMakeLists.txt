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
find_package(g2o REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(feature_extraction feature_extraction.cpp)
target_link_libraries(feature_extraction ${OpenCV_LIBS})

add_executable(epipolar_constraint_2d2d epipolar_constraint_2d2d.cpp)
target_link_libraries(epipolar_constraint_2d2d ${OpenCV_LIBS})

add_executable(pnp_3d2d pnp_3d2d.cpp)
target_link_libraries(pnp_3d2d ${OpenCV_LIBS} fmt g2o_core g2o_stuff)

add_executable(icp_3d3d icp_3d3d.cpp)
target_link_libraries(icp_3d3d ${OpenCV_LIBS} fmt g2o_core g2o_stuff)

