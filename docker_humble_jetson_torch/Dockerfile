FROM ghcr.io/epflxplore/docker_commons:humble-jetson-torch

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
RUN sed -i '3iset(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)' $REALSENSE_DIR/CMakeLists.txt
# Set the flags so that librealsense is compatible with python

# RUN cmake $REALSENSE_DIR/ -DBUILD_PYTHON_BINDINGS:bool=true -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true -DFORCE_RSUSB_BACKEND=ON  -DBUILD_WITH_CUDA:bool=true
RUN cmake $REALSENSE_DIR/ -DBUILD_PYTHON_BINDINGS:bool=true -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true -DFORCE_RSUSB_BACKEND=ON

RUN sudo make uninstall && sudo make clean

RUN echo "[INFO] Building is starting, it will take a long time like half an hour or more!"
RUN sleep 2
RUN sudo make -j$(($(nproc)-1)) && sudo make install


# Downloads to user config dir for Ultralytics
ADD https://github.com/ultralytics/assets/releases/download/v0.0.0/Arial.ttf \
    https://github.com/ultralytics/assets/releases/download/v0.0.0/Arial.Unicode.ttf \
    /root/.config/Ultralytics/

# Install linux packages for Ultralytics
# g++ required to build 'tflite_support' and 'lap' packages
# libusb-1.0-0 required for 'tflite_support' package when exporting to TFLite
# pkg-config and libhdf5-dev (not included) are needed to build 'h5py==3.11.0' aarch64 wheel required by 'tensorflow'
RUN apt update \
    && apt install --no-install-recommends -y gcc git zip unzip  wget curl htop libgl1 libglib2.0-0 libpython3-dev gnupg g++ libusb-1.0-0

# Create working directory
WORKDIR /ultralytics

# Copy contents and assign permissions for Ultralytics
COPY . .    
# RUN git clone https://github.com/ultralytics/ultralytics
RUN git remote set-url origin https://github.com/ultralytics/ultralytics.git
ADD https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8n.pt .

# RUN pwd
# RUN ls 
# RUN ls ultralytics/

# Remove opencv-python from Ultralytics dependencies as it conflicts with opencv-python installed in base image
# RUN grep -v "opencv-python" pyproject.toml > temp.toml && mv temp.toml pyproject.toml

# Download onnxruntime-gpu 1.15.1 for Jetson Linux 35.2.1 (JetPack 5.1). Other versions can be seen in https://elinux.org/Jetson_Zoo#ONNX_Runtime
ADD https://nvidia.box.com/shared/static/mvdcltm9ewdy2d5nurkiqorofz1s53ww.whl onnxruntime_gpu-1.15.1-cp38-cp38-linux_aarch64.whl



# Install ROS 2 HD packages with RosInstall to install from source
RUN apt-get update && apt-get upgrade -y
RUN apt update && sudo apt upgrade -y
RUN /home/ros2_install.sh \
    # moveit \
    depthai-ros
    # hardware_interface \
    # control_toolbox \
    # moveit_servo \
    # controller_manager
    
RUN apt-get install -y --no-install-recommends \
    python3-zmq \
    libyaml-cpp-dev \
    lcov

  # Add USB rules
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
RUN /etc/init.d/udev restart  



# Ensure Python and pip are installed and updated
RUN pip install --no-cache-dir --upgrade pip

# Install the necessary Python packageslibrealsense
RUN pip install --no-cache-dir -v evdev 
# pyrealsense2 wrapper has to be build from source 

# Install Ultralytics pip packages manually for TensorRT compatibility https://github.com/NVIDIA/TensorRT/issues/2567
RUN python3 -m pip install --upgrade pip wheel
RUN pip install onnxruntime_gpu-1.15.1-cp38-cp38-linux_aarch64.whl
RUN pip install --no-cache-dir -e ".[export]"
RUN rm *.whl

USER $USERNAME



# Set a diretory to store the project
WORKDIR /home/$USERNAME/dev_ws/src
COPY . .

# Set a directory to build the project
WORKDIR /home/$USERNAME/dev_ws

# Clean up
RUN sudo rm -rf /var/lib/apt/lists/*

# Remove all the confidential Xplore source code from the image
RUN sudo rm -rf /home/$USERNAME/dev_ws/src/*
