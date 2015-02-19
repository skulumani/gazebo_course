#!/bin/bash
# script to set environment variables for Gazebo for Homework 1
# 20 Jan 2015 - Shankar Kulumani

export GAZEBO_MODEL_PATH="/Users/shankar/Drive/GWU/classes/CSCI6525/homework/homework_1/feedback_control"
export GAZEBO_PLUGIN_PATH="/Users/shankar/Drive/GWU/classes/CSCI6525/homework/homework_1/feedback_control/build"

cd ./build
cmake 
make
cd ..
# change the filename here for different experiments
gazebo -u --verbose track_sinusoidal.world

# plot the data output after stopping the gazebo sim
python plot_data.py

