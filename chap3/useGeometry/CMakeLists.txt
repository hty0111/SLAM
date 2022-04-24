cmake_minimum_required( VERSION 3.0 )
project( Geometry )

# 添加Eigen头文件
include_directories( "/usr/include/eigen3" )

add_executable( eigenGeometry eigenGeometry.cpp )