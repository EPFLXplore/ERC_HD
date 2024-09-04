###########################################
# Xplore Handling Device image
###########################################
FROM ghcr.io/epflxplore/docker_commons:humble-desktop-nvidia

ARG DEBIAN_FRONTEND=noninteractive

# Install ROS 2 HD packages
RUN apt-get update && apt-get upgrade -y



RUN sudo apt-get update
RUN apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-depthai-ros \
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-control-toolbox \
    ros-${ROS_DISTRO}-moveit-servo \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-gazebo-ros \
    python3-zmq \
    libyaml-cpp-dev \
    lcov \
    libssl-dev \
    libusb-1.0-0-dev \
    libudev-dev \
    pkg-config \
    libgtk-3-dev\
    wget \
    cmake\
    build-essential \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    at \
    unzip \
    build-essential \
    cmake \
    git 
# librealsense2-dkms \
# librealsense2-utils \
# librealsense2-dev \ 
# librealsense2-dbg 

RUN mkdir -p /etc/apt/keyrings
# RUN curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
# RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" 

# Install librealsense
RUN curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/librealsense.list
RUN apt-get update
RUN apt-get update
RUN apt-get install -y \
    librealsense2-dkms \
    librealsense2-utils
# for developer and debug packages
# librealsense2-dev
# librealsense2-dbg


# Add USB rules
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
RUN /etc/init.d/udev restart

USER $USERNAME

# RUN pip install --no-cache-dir --upgrade pip && \
#     pip install --no-cache-dir evdev  pyrealsense2 

# Ensure Python and pip are installed and updated
RUN pip install --no-cache-dir --upgrade pip

# Install the necessary Python packages
RUN pip install --no-cache-dir -v evdev pyrealsense2 ultralytics

# Set a diretory to store the project
WORKDIR /home/$USERNAME/dev_ws/src
COPY . .

# Set a directory to build the project
WORKDIR /home/$USERNAME/dev_ws

# Clean up
RUN sudo rm -rf /var/lib/apt/lists/*

# Remove all the confidential Xplore source code from the image
RUN sudo rm -rf /home/$USERNAME/dev_ws/src/*