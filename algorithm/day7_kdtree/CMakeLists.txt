cmake_minimum_required(VERSION 3.0)

project(kdtree_demo)


set(CMAKE_CXX_STANDARD 11)


find_package(PCL REQUIRED)


include_directories(
    ${PCL_INCLUDE_DIRS}
)


add_executable(
    kdtree_demo
    kdtree_demo.cpp
)


target_link_libraries(
    kdtree_demo
    ${PCL_LIBRARIES}
)
