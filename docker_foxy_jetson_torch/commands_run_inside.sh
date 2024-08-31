sudo apt-get update

sudo apt-get install vim

# from build folder
cmake ../ -DBUILD_PYTHON_BINDINGS:bool=true -DPYTHON_EXECUTABLE=/usr/bin/python3.8 -DCMAKE_BUILD_TYPE=release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true -DBUILD_WITH_CUDA:bool=true -DFORCE_RSUSB_BACKEND=ON
sudo make uninstall && make clean
make -j6 
sudo make install
