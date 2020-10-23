cmake_minimum_required(VERSION 2.8.3)
project(untitled)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        tf2
        std_msgs)
include_directories( "/usr/include/eigen3" )
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

#add_subdirectory(/home/xcy/my_install/dlib/dlib dlib_build)
add_executable(untitled main.cpp find_closest.h find_closest.cpp residual.h residual.cpp)