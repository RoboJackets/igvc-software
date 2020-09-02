#!/bin/bash
if [ ! -d "/usr/local/include/kindr/" ]; then
    git clone https://github.com/ANYbotics/kindr.git
    cd kindr
    mkdir build
    cd build
    cmake ..
    sudo make install
    cd ../..
    rm -rf kindr
fi
rosdep install -iy --from-paths ../../src --skip-keys='kindr cmake_code_coverage cmake_clang_tools python-serial python-catkin-pkg python-pytorch-pip'
pip3 install --no-cache-dir torch torchvision
