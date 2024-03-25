FROM ros:humble

# Housekeeping
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get upgrade -y
RUN ln -s /usr/bin/python3 /usr/bin/python

# Install required dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        software-properties-common \
        ca-certificates \
        python2 \
        python-pip \
        git \
        libx11-6 \
        libxext-dev \
        libxrender-dev \
        libxtst-dev \
        libcanberra-gtk-module \
        libxshmfence1 libglu1 \
        libnss3-dev libgdk-pixbuf2.0-dev libgtk-3-dev libxss-dev libsecret-1-dev \
        libcanberra-gtk3-module x11-apps \
        wget \
        libfuse2 \
        unzip

# Installing arduino IDE
WORKDIR /root

RUN wget -O arduino-ide.zip https://downloads.arduino.cc/arduino-ide/arduino-ide_2.3.2_Linux_64bit.zip
RUN unzip arduino-ide.zip
RUN mv arduino-ide_2.3.2_Linux_64bit arduino-ide && rm arduino-ide.zip

# Installing arduino libs manager
WORKDIR /root/arduino-ide/

RUN wget -O arduino-cli.tar.gz https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Linux_64bit.tar.gz
RUN tar -xf arduino-cli.tar.gz && rm arduino-cli.tar.gz

# Installing esp-32 board
RUN ./arduino-cli config init --additional-urls https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
RUN ./arduino-cli core update-index --config-file arduino-cli.yaml && ./arduino-cli core install esp32:esp32
RUN wget -O get-pip.py https://bootstrap.pypa.io/get-pip.py && python get-pip.py && pip install pyserial && rm get-pip.py

# Installing micro ros Arduino
COPY micro_ros_arduino.zip micro-ros.zip
RUN ./arduino-cli config set library.enable_unsafe_install true
RUN ./arduino-cli lib install --zip-path micro-ros.zip && rm micro-ros.zip

# Installing extra Arduino libraries
RUN ./arduino-cli lib install ESP32Servo

# Getting and configuring terminal and env vars
COPY .bashrc_ex /root/.bashrc 
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /root/ball-beam/install/local_setup.bash" >> /root/.bashrc
RUN echo "alias arduino='/root/arduino-ide/arduino-ide --no-sandbox'" >> /root/.bashrc
ENV DISPLAY :0

# Installing python dependencies
RUN pip install matplotlib scipy

# Final configs
WORKDIR /root/ball-beam/
CMD ["bash"]