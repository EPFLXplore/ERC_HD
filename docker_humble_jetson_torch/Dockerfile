FROM ghcr.io/epflxplore/docker_commons:humble-jetson-torch

# Avoid interactive dialogue from apt-get
ENV DEBIAN_FRONTEND=noninteractive
ENV REALSENSE_BASE=/root
ENV REALSENSE_DIR=$REALSENSE_BASE/librealsense

# Combine related commands into single RUN statement
RUN sudo apt-get update && sudo apt-get upgrade -y \
    && sudo apt-get autoremove -y && sudo apt-get clean -y \
    && sudo apt-get install -y \
       python3 python3-dev libssl-dev libxinerama-dev libxcursor-dev libcanberra-gtk-module libcanberra-gtk3-module \
       libusb-1.0-0-dev pkg-config libgtk-3-dev python3-numpy \
       gcc git zip unzip wget curl htop libgl1 libglib2.0-0 libpython3-dev gnupg g++ libusb-1.0-0 \
       python3-zmq libyaml-cpp-dev lcov \
    && rm -rf /var/lib/apt/lists/* \
    && git clone https://github.com/IntelRealSense/librealsense.git $REALSENSE_DIR \
    && cd $REALSENSE_DIR && mkdir build \
    && sed -i '3iset(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)' $REALSENSE_DIR/CMakeLists.txt \
    && cmake $REALSENSE_DIR/ -DBUILD_PYTHON_BINDINGS:bool=true -DPYTHON_EXECUTABLE=/usr/bin/python3 \
       -DCMAKE_BUILD_TYPE=release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true -DFORCE_RSUSB_BACKEND=ON \
    && sudo make -j$(($(nproc)-1)) && sudo make install \
    && git clone https://github.com/ultralytics/ultralytics.git /home/$USERNAME/ultralytics \
    && cd /home/$USERNAME/ultralytics \
    && grep -v "opencv-python" pyproject.toml > temp.toml && mv temp.toml pyproject.toml \
    && wget https://nvidia.box.com/shared/static/mvdcltm9ewdy2d5nurkiqorofz1s53ww.whl -O onnxruntime_gpu-1.15.1-cp38-cp38-linux_aarch64.whl \
    && pip install --no-cache-dir --upgrade pip \
    && pip install --no-cache-dir -v evdev \
    && pip install onnxruntime_gpu-1.15.1-cp38-cp38-linux_aarch64.whl \
    && pip install --no-cache-dir -e ".[export]" \
    && rm *.whl

# Add USB rules and restart udev
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules \
    && sudo /etc/init.d/udev restart

USER $USERNAME

# Set directories
WORKDIR /home/$USERNAME/dev_ws/src
COPY . .

WORKDIR /home/$USERNAME/dev_ws

# Clean up
RUN sudo rm -rf /var/lib/apt/lists/* /home/$USERNAME/dev_ws/src/*
