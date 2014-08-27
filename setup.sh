#! /bin/sh
echo "Installing build-essential..."
sudo apt-get install build-essential


echo "Installing the Point Cloud Library..."
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install libpcl-all


if [ ! -f /usr/local/include/sicklms-1.0/SickLIDAR.hh ]; then
    echo "Installing the SICK ToolBox..."
    # Eventually change it to download the pre-modified tarball:
    # https://owncloud.robojackets.org/public.php?service=files&t=f68716a1e61616aab79297cc6f593eb9
    wget http://downloads.sourceforge.net/project/sicktoolbox/sicktoolbox/1.0.1/sicktoolbox-1.0.1.tar.gz ~/Downloads/
    tar -zxvf Downloads/sicktoolbox-1.0.1.tar.gz -C ~/
    cd ~/sicktoolbox-1.0.1/
    ./configure
    # Add includes that prevent the code from building
    sed -i -e '22i#include <unistd.h>\' c++/drivers/base/src/SickBufferMonitor.hh
    sed -i -e '42i#include <unistd.h>\' c++/drivers/base/src/SickLIDAR.hh
    make && sudo make install
else
    echo "SICK ToolBox seems to already be installed, so I'll skip that step."
fi


echo "Installing OpenCV..."
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install libopencv-dev


echo "Installing Qt..."
wget http://download.qt-project.org/official_releases/online_installers/qt-opensource-linux-x64-1.6.0-4-online.run -O ~/Downloads/qt-opensource-linux-x64-1.6.0-4-online.run
cd ~/Downloads
sudo chmod +x qt-opensource-linux-x64-1.6.0-4-online.run
sudo ./qt-opensource-linux-x64-1.6.0-4-online.run


# Evetually add automatic PTGrey installation
sudo apt-get install libglademm-2.4-1c2a libgtkglextmm-x11-1.2-dev
echo "######################################################"
echo "Please download and install FlyCapture from ptgrey.com"
echo "######################################################"


