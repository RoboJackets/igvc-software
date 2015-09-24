#!/bin/bash

# A small script to automate installation of dependencies

# cd to project root
cd ../../

catkin_make || true
apt-get update
source devel/setup.sh
rosdep -y install igvc
apt-get -y install qt5-default wget
wget https://files.gitter.im/RoboJackets/igvc-software/BGRW/flycapture2-2.8.3.1-amd64-pkg.tgz
tar -xvf flycapture2-2.8.3.1-amd64-pkg.tgz
cd flycapture2-2.8.3.1
yes |./install_flycapture.sh || true
apt-get install -yf
yes | ./install_flycapture.sh || true
apt-get install -yf
yes | ./install_flycapture.sh || true
