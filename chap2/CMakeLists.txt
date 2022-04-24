cmake_minimum_required( VERSION 3.0)

project( HelloSLAM )

add_executable( helloSLAM helloSLAM.cpp )

add_library( hello libHelloSLAM.cpp )

target_link_libraries( helloSLAM hello )