#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Define environment variables
export DEBIAN_FRONTEND=noninteractive
export REALSENSE_BASE=/root
export REALSENSE_DIR=$REALSENSE_BASE/librealsense

# Update and upgrade system packages
sudo apt-get update && sudo apt-get -y upgrade

# Install necessary dependencies including Python 3 development headers and libraries
sudo apt-get install -y \
    python3 python3-dev python3-pip python3-distutils libssl-dev libxinerama-dev libxcursor-dev \
    libcanberra-gtk-module libcanberra-gtk3-module libusb-1.0-0-dev \
    pkg-config libgtk-3-dev python3-numpy cmake

# Upgrade pip
python3 -m pip install --upgrade pip

# Check if the librealsense directory exists and delete it if it does
if [ -d "$REALSENSE_DIR" ]; then
    echo "[INFO] Directory $REALSENSE_DIR already exists. Deleting..."
    sudo rm -rf $REALSENSE_DIR
fi

# Clone librealsense SDK
git clone https://github.com/IntelRealSense/librealsense.git $REALSENSE_DIR
cd $REALSENSE_DIR
mkdir build
cd build

# Give the CUDA path to CMake if CUDA is being used
sed -i '3iset(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)' $REALSENSE_DIR/CMakeLists.txt

# Run CMake with the specified flags and explicitly specify Python 3 paths
cmake $REALSENSE_DIR/ \
    -DBUILD_PYTHON_BINDINGS:bool=true \
    -DPYTHON_EXECUTABLE=$(which python3) \
    -DPYTHON_INCLUDE_DIR=$(python3 -c "from sysconfig import get_paths as gp; print(gp()['include'])") \
    -DPYTHON_LIBRARY=$(python3 -c "from distutils.sysconfig import get_config_var; print(get_config_var('LIBDIR'))") \
    -DCMAKE_BUILD_TYPE=release \
    -DBUILD_EXAMPLES=true \
    -DBUILD_GRAPHICAL_EXAMPLES=true \
    -DFORCE_RSUSB_BACKEND=ON

# Clean previous builds if any
sudo make uninstall || true
sudo make clean || true

# Build and install librealsense
echo "[INFO] Building is starting, it might take some time!"
sudo make -j$(($(nproc)-1))
sudo make install

# Clean up
sudo rm -rf /var/lib/apt/lists/*
