#!/bin/bash

CUSTOM_PKG_MICROROS=ball_beam_msgs
BALL_BEAM_TOP_DIR=$(git rev-parse --show-toplevel)

# Copy custom msgs pkg to microros folder
mkdir -p $BALL_BEAM_TOP_DIR/micro_ros_arduino/extras/library_generation/extra_packages/$CUSTOM_PKG_MICROROS
cp -r $BALL_BEAM_TOP_DIR/src/$CUSTOM_PKG_MICROROS/*  $BALL_BEAM_TOP_DIR/micro_ros_arduino/extras/library_generation/extra_packages/$CUSTOM_PKG_MICROROS/

# Build custom messages
cd $BALL_BEAM_TOP_DIR/micro_ros_arduino
docker run -it --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=extras microros/micro_ros_static_library_builder:humble -p esp32

# Zip builded state of micro_ros_arduino
cd ../
zip -r utils/build_scripts/micro_ros_arduino.zip micro_ros_arduino

# Build ball-beam docker image
cd utils/build_scripts/
docker build . -t ball-beam:latest

# Remove unnecessary file
rm -f micro_ros_arduino.zip
sudo chown -R $USER $BALL_BEAM_TOP_DIR