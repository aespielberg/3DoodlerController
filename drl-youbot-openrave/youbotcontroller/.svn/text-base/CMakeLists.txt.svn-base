cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
add_definitions(-Wall)
rosbuild_init()

add_definitions(-Wall)

rosbuild_add_boost_directories()

rosbuild_add_library(youbotcontroller youbotcontroller_interface.cpp youbotcontroller.cpp plugindefs.h)

