#!/bin/bash

CUSTOM_PKG_MICROROS=ball_beam_msgs

# Copy custom msgs pkg to microros folder
mkdir -p ../micro_ros_arduino/extras/library_generation/extra_packages/$CUSTOM_PKG_MICROROS
cp -r ../src/$CUSTOM_PKG_MICROROS/*  ../micro_ros_arduino/extras/library_generation/extra_packages/$CUSTOM_PKG_MICROROS/

# # Build custom messages
cd ../micro_ros_arduino
#docker run -it --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=extras microros/micro_ros_static_library_builder:humble -p esp32
cd ..
zip -r utils/micro_ros_arduino.zip micro_ros_arduino

# Pull micro ros agent docker image
docker pull microros/micro-ros-agent:humble

# Build ball-beam docker image
cd utils/
docker build . -t ball-beam:latest

# Remove unnecessary file
rm -f micro_ros_arduino.zip