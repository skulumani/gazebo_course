#!/bin/bash
# script to set environment variables for Gazebo for Homework 4
# 10 Mar 2015 - Shankar Kulumani

export GAZEBO_MODEL_PATH="/home/shankar/CSCI6525/gazebo_course/IK"
export GAZEBO_PLUGIN_PATH="/home/shankar/CSCI6525/gazebo_course/IK/debug"
export LD_LIBRARY_PATH="/home/shankar/CSCI6525/gazebo_course/Ravelin/build"

# cd ./build
# cmake ..
# debug options
cd ./debug
# add debug flag to cmake
cmake -DCMAKE_BUILD_TYPE=Debug
make
cd ..
# run gazebo in debug mode
# gdb --args gzserver --verbose planar.world
# once in gdb type run to run planar.world
# it will fail at some point. use backtrace to look at frames and find where the error comes from
# frame # lets you jump to a number
# break IKPlanar.cpp:207 places  break point at line 207
# next and step can be used to go line by line or into a function
# print lets you print variables (hard to print arrays/poses)

# change the filename here for different experiments
# gazebo -u --verbose planar2.world
gzserver --verbose planar.world



