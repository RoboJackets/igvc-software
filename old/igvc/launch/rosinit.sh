#!/bin/bash
#ln -s /home/robojackets/catkin_ws/src/igvc-software/igvc/launch/rosinit.sh /#etc/init.d/
export ROS_MASTER_URI=http://10.0.0.1:11311
export ROS_IP=`hostname -I`
echo $ROS_IP
roslaunch igvc sidecameras.launch & roslaunch igvc sidelinedetectors.launch
