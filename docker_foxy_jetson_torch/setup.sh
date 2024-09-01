# sudo apt-get update

# sudo apt-get install vim

# # from build folder
# cmake ../ -DBUILD_PYTHON_BINDINGS:bool=true -DPYTHON_EXECUTABLE=/usr/bin/python3.8 -DCMAKE_BUILD_TYPE=release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true -DBUILD_WITH_CUDA:bool=true -DFORCE_RSUSB_BACKEND=ON
# sudo make uninstall && make clean
# make -j6 
# sudo make install


#!/bin/bash

# Source ROS 2 environment
source /opt/ros/foxy/setup.bash

# Change to the home directory of the current user
cd /home/$USERNAME

# Clone the vision_opencv repository from GitHub, using the 'foxy' branch
git clone https://github.com/ros-perception/vision_opencv.git -b foxy

# Change directory to the cloned repository
cd vision_opencv

# Update the package lists for upgrades, new packages, and package upgrades
sudo apt update

# Install OpenCV development libraries
sudo apt install -y libopencv-dev ros-foxy-ament-cmake mlocate

# Build the ROS package with colcon, using symlink install
colcon build --symlink-install

# # (Optional) Set OpenCV_DIR environment variable
export OpenCV_DIR=/usr/lib/aarch64-linux-gnu/cmake/opencv4/OpenCVConfig.cmake

# # (Optional) Install mlocate and locate OpenCVConfig.cmake file
# # Uncomment the following lines if you need to install mlocate and locate OpenCVConfig.cmake
# # sudo apt install -y mlocate
# # locate OpenCVConfig.cmake

# # Install mlocate
# sudo apt install -y mlocate

# # Update mlocate database
# sudo updatedb

sudo apt install ros-foxy-cv-bridge

# # Locate the OpenCVConfig.cmake file and save the result
# opencv_config_path=$(locate OpenCVConfig.cmake | grep '/usr/lib/aarch64-linux-gnu/cmake/opencv4/OpenCVConfig.cmake' | head -n 1)

# # Check if the locate command found the file and set the OpenCV_DIR environment variable
# if [ -n "$opencv_config_path" ]; then
#   export OpenCV_DIR="$opencv_config_path"
#   echo "OpenCV_DIR set to $OpenCV_DIR"
# else
#   echo "OpenCVConfig.cmake not found. Please check the path."
# fi

# Source the setup.bash files
source ~/vision_opencv/install/setup.bash
source ~/dev_ws/install/setup.bash

# Change directory to the ROS workspace
cd ~/dev_ws

# Build the ROS workspace
colcon build --packages-select hd_launch perception custom_msg

source install/setup.bash

