#!/bin/bash
# script to set environment variables for Gazebo for Homework 5
# 2 Apr 2015 - Shankar Kulumani

export GAZEBO_MODEL_PATH="/home/shankar/CSCI6525/gazebo_course/inverse_dynamics"
export GAZEBO_PLUGIN_PATH="/home/shankar/CSCI6525/gazebo_course/inverse_dynamics/build"
export LD_LIBRARY_PATH="/home/shankar/CSCI6525/Ravelin/build"

cd ./build
 cmake ..
# debug options
# cd ./debug
# add debug flag to cmake
#cmake -DCMAKE_BUILD_TYPE=Debug ..
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
# gdb --args gzserver --verbose spatial.world
#gazebo -u --verbose spatial.world
gazebo -u --verbose track_sinusoidal.world



