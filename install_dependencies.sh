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
# manually install nmea-navsat-driver because it won't work otherwise. FIX EVENTUALLY.
sudo apt-get install ros-noetic-nmea-navsat-driver
rosdep install -iy --from-paths ../../src --skip-keys='kindr nmea-navsat-driver'
pip3 install --no-cache-dir torch torchvision
