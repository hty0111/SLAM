cmake_minimum_required(VERSION 3.0)
project(PROJECT)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
include_directories(${PROJECT_SOURCE_DIR}/include)

find_package(PCL REQUIRED COMPONENTS)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcd_write src/pcd_write.cpp)
target_link_libraries(pcd_write ${PCL_LIBRARIES})

add_executable(matrix_transform src/matrix_transform.cpp)
target_link_libraries(matrix_transform ${PCL_LIBRARIES})

add_executable(normal_estimation src/normal_estimation.cpp)
target_link_libraries(normal_estimation ${PCL_LIBRARIES})
