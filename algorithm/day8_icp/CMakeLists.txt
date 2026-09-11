cmake_minimum_required(VERSION 3.0)

project(day8_icp)


set(CMAKE_CXX_STANDARD 11)


find_package(PCL REQUIRED)


include_directories(
    ${PCL_INCLUDE_DIRS}
)


add_executable(
    icp_demo
    icp_demo.cpp
)


target_link_libraries(
    icp_demo
    ${PCL_LIBRARIES}
)
