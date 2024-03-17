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
         -e DISPLAY=$DISPLAY \
         --privileged \
         ball-beam:latest  

# Stop micro ros agent container
docker stop micro_ros_agent

# Unallows root
xhost -SI:localuser:root