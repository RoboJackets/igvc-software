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

rosdep install -iy --from-paths ../../src --skip-keys='kindr serial'
pip3 install --no-cache-dir torch torchvision
