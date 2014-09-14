#! /bin/sh
echo "Installing build-essential..."
sudo apt-get install build-essential



echo "Installing OpenCV..."
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install libopencv-dev



if [ ! -f /usr/local/include/sicklms-1.0/SickLIDAR.hh ]; then
    echo "Installing the SICK ToolBox..."
    wget https://www.dropbox.com/s/i4nwdzhvggk5h39/sicktoolbox-1.0.1-MODIFIED.tar.gz?dl=0 -O ~/Downloads/sicktoolbox-1.0.1-MODIFIED.tar.gz
    tar -zxvf ~/Downloads/sicktoolbox-1.0.1-MODIFIED.tar.gz -C ~/
    cd ~/sicktoolbox-1.0.1/
    ./configure
    make && sudo make install
else
    echo "SICK ToolBox seems to already be installed, so I'll skip that step."
fi



echo "Installing PointGrey FlyCapture2..."
sudo apt-get install libglademm-2.4-1c2a libgtkglextmm-x11-1.2-dev
read -p "Are you running a 64-bit machine? (Select no if you are running a 32-bit machine.) (y/n):" yn
case $yn in
    [Nn]* ) wget https://www.dropbox.com/s/r259tr3r726edml/flycapture2-2.6.3.4-i386-pkg.tgz?dl=0 -O ~/Downloads/flycapture2-2.6.3.4-i386-pkg.tgz
            tar -zxvf ~/Downloads/flycapture2-2.6.3.4-i386-pkg.tgz -C ~/
            cd ~/flycapture2-2.6.3.4-i386
            ./install_flycapture.sh
    [Yy]* ) wget https://www.dropbox.com/s/aq3f484rbu3xmcx/flycapture2-2.6.3.4-amd64-pkg.tgz?dl=0 -O ~/Downloads/flycapture2-2.6.3.4-amd64-pkg.tgz
            tar -zxvf ~/Downloads/flycapture2-2.6.3.4-amd64-pkg.tgz -C ~/
            cd ~/flycapture2-2.6.3.4-amd64
            ./install_flycapture.sh
    * ) echo "Please answer yes or no,";;
esac



echo "Installing Qt..."
wget http://download.qt-project.org/official_releases/online_installers/qt-opensource-linux-x64-1.6.0-4-online.run -O ~/Downloads/qt-opensource-linux-x64-1.6.0-4-online.run
cd ~/Downloads
sudo chmod +x qt-opensource-linux-x64-1.6.0-4-online.run
sudo ./qt-opensource-linux-x64-1.6.0-4-online.run



echo "Installing the Point Cloud Library..."
read -p "NOTE: This will build PCL from source and can take a real long time. Are you sure you want to do this now? (y/n):" yn
case $yn in
    [Nn]* ) continue;;
    [Yy]* ) wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.7.1.zip -O ~/Downloads/pcl-1.7.1.zip
            cd ~/Downloads
            unzip pcl-1.7.1.zip -d ~/pcl
            cd ~/pcl/pcl-pcl-1.7.1
            mkdir build
            cd build
            cmake -D CMAKE_INSTALL_PREFIX=/usr ..
            make && sudo make install;;
    * ) echo "Please answer yes or no.";;
esac
