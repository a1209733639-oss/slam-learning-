cmake_minimum_required(VERSION 3.0)

project(day9_ndt)


set(CMAKE_CXX_STANDARD 11)


find_package(PCL REQUIRED)


include_directories(
${PCL_INCLUDE_DIRS}
)


add_executable(
ndt_demo
ndt_demo.cpp
)


target_link_libraries(
ndt_demo
${PCL_LIBRARIES}
)
