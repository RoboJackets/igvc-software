#!/bin/bash
# A small script to automate installation of dependencies


set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd "$DIR/../.."

catkin_make || true
sudo apt-get update
source ./devel/setup.sh
rosdep update
rosdep -y install igvc
sudo apt-get -y install qt5-default wget expect
wget https://files.gitter.im/RoboJackets/igvc-software/BGRW/flycapture2-2.8.3.1-amd64-pkg.tgz
tar -xvf flycapture2-2.8.3.1-amd64-pkg.tgz
cp ./src/util/flycapture-setup.exp flycapture2-2.8.3.1-amd64/
cd flycapture2-2.8.3.1-amd64


# Install dependencies for flycapture
sudo apt-get -y install libatkmm-1.6-1 libcairomm-1.0-1 libglade2-0 libglademm-2.4-1c2a libglibmm-2.4-1c2a \
	libgtkmm-2.4-1c2a libpangomm-1.4-1 libsigc++-2.0-0c2a libatkmm-1.6-dev libcairomm-1.0-dev libglibmm-2.4-dev \
	libgtkglext1-dev libgtkglextmm-x11-1.2-0 libgtkglextmm-x11-1.2-dev libgtkmm-2.4-dev libpangomm-1.4-dev \
	libpangox-1.0-dev libsigc++-2.0-dev libxmu-dev libxmu-headers

sudo ./flycapture-setup.exp			# Cross our fingers...
