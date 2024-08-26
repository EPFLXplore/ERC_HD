FROM ultralytics/ultralytics:latest-jetson-jetpack5

# Avoid interactive dialogue from apt-get
ENV DEBIAN_FRONTEND=noninteractive

# Add USB rules and restart udev
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules \
    && sudo /etc/init.d/udev restart

# Install ROS from 
RUN apt update && apt install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

RUN apt install -y software-properties-common
RUN add-apt-repository universe

RUN apt update && apt install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN sudo apt-get remove -y --purge libopencv opencv-licenses opencv-libs libopencv-dev \
    && sudo apt-get -y autoremove \
    && sudo apt-get -y clean

RUN sudo apt-get remove -y --purge opencv-dev

RUN apt update && apt upgrade -y \
    && apt install -y ros-foxy-desktop python3-argcomplete

USER $USERNAME

RUN . /opt/ros/foxy/setup.bash

# Set directories
WORKDIR /home/$USERNAME/dev_ws/src
COPY . .

WORKDIR /home/$USERNAME/dev_ws

# Clean up
RUN sudo rm -rf /var/lib/apt/lists/* /home/$USERNAME/dev_ws/src/*
