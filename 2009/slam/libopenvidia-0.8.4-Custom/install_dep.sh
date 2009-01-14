#!/bin/sh


# Listed Dependencies
#    * imlib-dev Imlib development libraries
#    * libxml-dev
#    * libdc1394-dev (v 1.x)
#        (and 1394 support (raw1394 or video1394))
#    * jpeg-mmx
#    * libfltk1.1
#    * libfltk1.1-dev
#    * libcommoncpp2-1.0-0c102
#    * libcommoncpp2-dev
#    * glut	
#    * nvidia-cg-toolkit

echo "misc ===================================================================="
sudo apt-get --yes install libcommoncpp2-dev
sudo apt-get --yes install nvidia-cg-toolkit
#sudo apt-get --yes install imlib-dev libxml-dev 

echo "glut ====================================================================" 
sudo apt-get --yes install libglut-dev

echo "opencv =================================================================="
sudo apt-get --yes install libcv1
sudo apt-get --yes install libcvaux1 
sudo apt-get --yes install libhighgui1
sudo apt-get --yes install libcv-dev

echo "g++ ====================================================================="
sudo apt-get --yes install g++





