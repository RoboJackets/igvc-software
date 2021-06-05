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

## GTSAM
# Add PPA
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt update  # not necessary since Bionic
# Install:
sudo apt install libgtsam-dev libgtsam-unstable-dev

## GeographicLib
sudo apt-get install -y libgeographic-dev
sudo ln -s /usr/share/cmake/geographiclib/FindGeographicLib.cmake /usr/share/cmake-3.16/Modules/