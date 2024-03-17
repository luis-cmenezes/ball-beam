#!/bin/bash

XSOCK=/tmp/.X11-unix

# Allows root
xhost +SI:localuser:root

# Run, in a separate non-interactive terminal, micro ros agent
gnome-terminal -- docker run --rm -v /dev:/dev \
                    --privileged --net=host \
                    --name micro_ros_agent \
                    microros/micro-ros-agent:humble \
                    serial --dev /dev/ttyUSB0 -v6

# Run, interactively in current terminal, ball-bean container
docker run -it --rm --net=host \
         -v $XSOCK:$XSOCK:ro \
         -v /dev:/dev \
         -v $(git rev-parse --show-toplevel)/src:/root/ball-beam/src/ \
         -v $(git rev-parse --show-toplevel)/install:/root/ball-beam/install/ \
         -v $(git rev-parse --show-toplevel)/build:/root/ball-beam/build/ \
         -e DISPLAY=$DISPLAY \
         --privileged \
         ball-beam:latest  

# Stop micro ros agent container
docker stop micro_ros_agent

# Unallows root
xhost -SI:localuser:root