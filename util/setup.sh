#!/bin/bash
# A small script to automate installation of dependencies



DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd "$DIR/../.."

catkin_make || true
sudo apt-get update
source ./devel/setup.sh
rosdep update
rosdep -y install igvc
sudo apt-get -y install qt5-default