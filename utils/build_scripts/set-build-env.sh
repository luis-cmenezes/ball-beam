#!/bin/bash

BALL_BEAM_TOP_DIR=$(git rev-parse --show-toplevel)

# Pull micro ros library builder image 
docker pull microros/micro_ros_static_library_builder:humble

# Pull micro ros agent docker image
docker pull microros/micro-ros-agent:humble

# Zip current state of micro_ros_arduino
cd ../../
zip -r utils/build_scripts/micro_ros_arduino.zip micro_ros_arduino

# Build ball-beam docker image
cd utils/build_scripts
docker build . -t ball-beam:latest
     
# Remove unnecessary file
rm -f micro_ros_arduino.zip
sudo chown -R $USER $BALL_BEAM_TOP_DIR