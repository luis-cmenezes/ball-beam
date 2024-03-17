#!/bin/bash

CUSTOM_PKG_MICROROS=ball_beam_msgs
BALL_BEAM_ESP_PATH=src/test/

# Removing existing build and install folders
sudo rm -rf ../build/ ../install/

# Copy custom msgs pkg to microros folder
mkdir -p ../micro_ros_arduino/extras/library_generation/extra_packages/$CUSTOM_PKG_MICROROS
cp -r ../src/$CUSTOM_PKG_MICROROS/*  ../micro_ros_arduino/extras/library_generation/extra_packages/$CUSTOM_PKG_MICROROS/

# Build custom messages
cd ../micro_ros_arduino
#docker run -it --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=extras microros/micro_ros_static_library_builder:humble -p esp32
cd ..
zip -r utils/micro_ros_arduino.zip micro_ros_arduino

# Pull micro ros agent docker image
docker pull microros/micro-ros-agent:humble

# Build ball-beam docker image
cd utils/
docker build . -t ball-beam:latest

# Build ball-beam ros2 source code
docker run --rm --net=host -it \
         -v $(git rev-parse --show-toplevel)/src:/root/ball-beam/src/ \
         -v $(git rev-parse --show-toplevel)/install:/root/ball-beam/install/ \
         -v $(git rev-parse --show-toplevel)/build:/root/ball-beam/build/ \
         --privileged \
         ball-beam:latest \
         bash -c "colcon build"

# Flash esp 32 code 
cp -r ../$BALL_BEAM_ESP_PATH ../src/esp_upload/

docker run --rm --net=host -it \
         -v $(git rev-parse --show-toplevel)/src:/root/ball-beam/src/ \
         --privileged \
         ball-beam:latest \
         bash -c "cd src/esp_upload/ &&  mv *.ino esp_upload.ino &&\
                  /root/arduino-ide/arduino-cli compile --fqbn \
                  esp32:esp32:nodemcu-32s . && \
                  /root/arduino-ide/arduino-cli upload -p \
                  /dev/ttyUSB0 --fqbn esp32:esp32:nodemcu-32s ."
     
# Remove unnecessary file
rm -rf ../src/esp_upload/
rm -f micro_ros_arduino.zip
sudo chown -R $USER ../