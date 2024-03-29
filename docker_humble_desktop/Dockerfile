###########################################
# Xplore Handling Device image
###########################################
FROM ghcr.io/epflxplore/docker_commons:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive

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

RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir evdev

# Set a diretory to store the project
WORKDIR /home/$USERNAME/dev_ws/src
COPY . .

# Set a directory to build the project
WORKDIR /home/$USERNAME/dev_ws

# Clean up
RUN sudo rm -rf /var/lib/apt/lists/*