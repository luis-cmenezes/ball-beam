#!/bin/bash

BALL_BEAM_TOP_DIR=$(git rev-parse --show-toplevel)

# Removing existing build and install folders
sudo rm -rf $BALL_BEAM_TOP_DIR/build/ $BALL_BEAM_TOP_DIR/install/

# Build ball-beam ros2 source code
docker run --rm --net=host -it \
         -v $BALL_BEAM_TOP_DIR/src:/root/ball-beam/src/ \
         -v $BALL_BEAM_TOP_DIR/install:/root/ball-beam/install/ \
         -v $BALL_BEAM_TOP_DIR/build:/root/ball-beam/build/ \
         --privileged \
         ball-beam:latest \
         bash -c "colcon build"
     
# Housekeeping
sudo chown -R $USER $BALL_BEAM_TOP_DIR