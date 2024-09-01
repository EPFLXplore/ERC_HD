
FROM ghcr.io/epflxplore/docker_commons:humble-jetson

# Avoid interactive dialogue from apt-get
ENV DEBIAN_FRONTEND=noninteractive

RUN sudo apt update && sudo apt autoremove -y && sudo apt clean -y

RUN sudo apt-get -y update && sudo apt-get -y upgrade \
    && echo "[INFO] Upgrades are done!" \
    && sleep 2

ENV REALSENSE_BASE=/root

ENV REALSENSE_DIR=$REALSENSE_BASE/librealsense

# clone librealsense SKD
RUN git clone https://github.com/IntelRealSense/librealsense.git $REALSENSE_DIR \
    && cd $REALSENSE_DIR \
    && mkdir build
 
RUN sudo apt-get -y install python3 python3-dev libssl-dev libxinerama-dev libxcursor-dev libcanberra-gtk-module libcanberra-gtk3-module
RUN apt-get update && apt-get install -y \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*
RUN echo "[INFO] pyrealsense2 bindings build has been started!"

# Give the CUDA path to CMake
RUN sed -i '3iset(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)\' $REALSENSE_DIR/CMakeLists.txt
# Set the flags so that librealsense is compatible with python

# RUN cmake $REALSENSE_DIR/ -DBUILD_PYTHON_BINDINGS:bool=true -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true -DFORCE_RSUSB_BACKEND=ON  -DBUILD_WITH_CUDA:bool=true
RUN cmake $REALSENSE_DIR/ -DBUILD_PYTHON_BINDINGS:bool=true -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true -DFORCE_RSUSB_BACKEND=ON

RUN sudo make uninstall && sudo make clean

RUN echo "[INFO] Building is starting, it will take a long time like half an hour or more!"
RUN sleep 2
RUN sudo make -j$(($(nproc)-1)) && sudo make install


# Install ROS 2 HD packages
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-depthai-ros \
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-control-toolbox \
    ros-${ROS_DISTRO}-moveit-servo \
    ros-${ROS_DISTRO}-controller-manager \
    python3-zmq \
    libyaml-cpp-dev \
    lcov

  # Add USB rules
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
RUN /etc/init.d/udev restart  


USER $USERNAME

# Ensure Python and pip are installed and updated
RUN pip install --no-cache-dir --upgrade pip

# Install the necessary Python packageslibrealsense
RUN pip install --no-cache-dir -v evdev 
# pyrealsense2 wrapper has to be build from source 

# MISSING pyrealsense2 

# Set a diretory to store the project
WORKDIR /home/$USERNAME/dev_ws/src
COPY . .

# Set a directory to build the project
WORKDIR /home/$USERNAME/dev_ws

# Clean up
RUN sudo rm -rf /var/lib/apt/lists/*

# Remove all the confidential Xplore source code from the image
RUN sudo rm -rf /home/$USERNAME/dev_ws/src/*