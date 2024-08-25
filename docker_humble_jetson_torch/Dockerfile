FROM ultralytics/ultralytics:latest-jetson-jetpack5

# Avoid interactive dialogue from apt-get
ENV DEBIAN_FRONTEND=noninteractive

# Add USB rules and restart udev
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules \
    && sudo /etc/init.d/udev restart

# Install ROS from 
RUN apt update && sudo apt install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

RUN apt install -y software-properties-common
RUN add-apt-repository universe

RUN apt update && sudo apt install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update && sudo apt install -y \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools

USER $USERNAME

RUN python3 -m pip install -U \
   flake8-blind-except \
   flake8-builtins \
   flake8-class-newline \
   flake8-comprehensions \
   flake8-deprecated \
   flake8-import-order \
   flake8-quotes \
   "pytest>=5.3" \
   pytest-repeat \
   pytest-rerunfailures

RUN mkdir -p ~/ros2_humble/src && cd ~/ros2_humble \
    && vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src

RUN sudo apt-get remove -y --purge libopencv opencv-licenses opencv-libs libopencv-dev \
    && sudo apt-get -y autoremove \
    && sudo apt-get -y clean

RUN sudo apt-get remove -y --purge opencv-dev

RUN cd ~/ros2_humble && sudo rosdep init \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -y --os=ubuntu:jammy --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

RUN cd ~/ros2_humble/ \
    && colcon build --symlink-install

RUN . ~/ros2_humble/install/local_setup.bash

# Set directories
WORKDIR /home/$USERNAME/dev_ws/src
COPY . .

WORKDIR /home/$USERNAME/dev_ws

# Clean up
RUN sudo rm -rf /var/lib/apt/lists/* /home/$USERNAME/dev_ws/src/*
