#!/bin/bash
# A small script to automate installation of dependencies


set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd "$DIR/../.."

catkin_make || true
apt-get update
source src/devel/setup.sh
rosdep -y install igvc
apt-get -y install qt5-default wget expect
wget https://files.gitter.im/RoboJackets/igvc-software/BGRW/flycapture2-2.8.3.1-amd64-pkg.tgz
tar -xvf flycapture2-2.8.3.1-amd64-pkg.tgz
cp ./src/util/flycapture-setup.exp flycapture2-2.8.3.1-amd64/
cd flycapture2-2.8.3.1-amd64

./flycapture-setup.exp || true		# Cross our fingers...
apt-get install -yf
./flycapture-setup.exp || true		# Cross our fingers...
apt-get install -yf
./flycapture-setup2.exp				# Cross our fingers...
