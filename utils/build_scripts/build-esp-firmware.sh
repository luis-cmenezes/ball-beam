#!/bin/bash

ESP_CODE_PKG=pot_calibration/
BALL_BEAM_TOP_DIR=$(git rev-parse --show-toplevel)

# Flash esp 32 code 
mkdir -p $BALL_BEAM_TOP_DIR/src/esp_upload/
cp $BALL_BEAM_TOP_DIR/src/$ESP_CODE_PKG/*.ino $BALL_BEAM_TOP_DIR/src/esp_upload/esp_upload.ino

docker run --rm --net=host -it \
         -v $BALL_BEAM_TOP_DIR/src/esp_upload/:/root/ball-beam/src/esp_upload/ \
         --privileged \
         ball-beam:latest \
         bash -c "cd src/esp_upload/ &&\
                  /root/arduino-ide/arduino-cli compile --fqbn \
                  esp32:esp32:nodemcu-32s . && \
                  /root/arduino-ide/arduino-cli upload -p \
                  /dev/ttyUSB0 --fqbn esp32:esp32:nodemcu-32s ."
     
# Remove unnecessary file
rm -rf $BALL_BEAM_TOP_DIR/src/esp_upload/
sudo chown -R $USER $BALL_BEAM_TOP_DIR