#!/bin/bash
# script to set environment variables for Gazebo for Homework 4
# 10 Mar 2015 - Shankar Kulumani

export GAZEBO_MODEL_PATH="/home/shankar/CSCI6525/gazebo_course/IK"
export GAZEBO_PLUGIN_PATH="/home/shankar/CSCI6525/gazebo_course/IK/build"
export LD_LIBRARY_PATH="/home/shankar/CSCI6525/gazebo_course/Ravelin/build"

cd ./build
cmake ..
make
cd ..
# change the filename here for different experiments
# gazebo -u --verbose planar2.world
gzserver --verbose planar.world


