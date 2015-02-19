#!/bin/bash
# script to set environment variables for Gazebo for Homework 2
# 2 Feb 2015 - Shankar Kulumani

export GAZEBO_MODEL_PATH="/media/sf_VirtualBox_Shared/coord_frames"
export GAZEBO_PLUGIN_PATH="/media/sf_VirtualBox_Shared/coord_frames/build"

cd ./build
cmake ..
make
cd ..
# change the filename here for different experiments
gazebo -u --verbose planar2.world


