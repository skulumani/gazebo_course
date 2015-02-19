#!/bin/bash
# script to set environment variables for Gazebo for Homework 3
# 17 Feb 2015 - Shankar Kulumani

export GAZEBO_MODEL_PATH="/home/shankar/CSCI6525/gazebo_course/Jacobians"
export GAZEBO_PLUGIN_PATH="/home/shankar/CSCI6525/gazebo_course/Jacobians/build"
export LD_LIBRARY_PATH="/home/shankar/CSCI6525/gazebo_course/Jacobians/Ravelin/build"

cd ./build
cmake ..
make
cd ..
# change the filename here for different experiments
# gazebo -u --verbose planar2.world
gzserver --verbose planar.world


