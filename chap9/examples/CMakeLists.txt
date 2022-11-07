cmake_minimum_required(VERSION 3.0)

project(bundle_adjustment)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -std=c++17")

if (POLICY CMP0072)
    set(OpenGL_GL_PREFERENCE LEGACY)
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

Find_Package(g2o REQUIRED)
Find_Package(Eigen3 REQUIRED)
Find_Package(Ceres REQUIRED)
Find_Package(Sophus REQUIRED)
Find_Package(CSparse REQUIRED)

SET(G2O_LIBS g2o_csparse_extension g2o_stuff g2o_core cxsparse fmt)

include_directories(${PROJECT_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR})

add_library(bal_common src/common.cpp)

add_executable(bundle_adjustment_g2o src/bundle_adjustment_g2o.cpp)
add_executable(bundle_adjustment_ceres src/bundle_adjustment_ceres.cpp)

target_link_libraries(bundle_adjustment_ceres ${CERES_LIBRARIES} bal_common)
target_link_libraries(bundle_adjustment_g2o ${G2O_LIBS} bal_common)
