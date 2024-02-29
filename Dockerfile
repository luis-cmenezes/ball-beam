FROM ros:humble

# Housekeeping
SHELL ["/bin/bash", "-c"]
RUN apt-get update
RUN apt-get install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0 -y
RUN ln -s /usr/bin/python3 /usr/bin/python

# Installing micro-ros
RUN mkdir -p ~/microros_ws

WORKDIR /root/microros_ws/
    
RUN git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
RUN apt update && rosdep update
RUN rosdep install --from-path src --ignore-src -y
RUN apt-get install python3-pip
RUN pip install catkin_pkg
RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon build

# Creating firmware ws for ESP32 in ARG ws_name
ARG ws_name=ball-beam

RUN mkdir -p ~/$ws_name

WORKDIR /root/$ws_name

RUN source /root/microros_ws/install/local_setup.bash && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    ros2 run micro_ros_setup create_firmware_ws.sh freertos esp32

WORKDIR /root/$ws_name/firmware/toolchain/esp-idf

RUN ./install.sh

# Getting, configuring and coloring terminal
ENV TERM=xterm-256color

RUN cp /etc/bash.bashrc /root/.bashrc

RUN echo "PS1='\e[92m\u\e[0m@\e[94m\h\e[0m:\e[35m\w\e[0m# '" >> /root/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source ~/microros_ws/install/local_setup.bash" >> /root/.bashrc
RUN echo "alias get_idf='. /root/$ws_name/firmware/toolchain/esp-idf/export.sh'" >> /root/.bashrc
RUN echo "get_idf" >> /root/.bashrc

WORKDIR /root/$ws_name

CMD ["bash"]