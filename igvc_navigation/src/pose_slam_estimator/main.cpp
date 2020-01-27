/**
Author: Alejandro Escontrela <aescontrela3@gatech.edu>
Date Created: January 22nd, 2020
*/

#include "PoseSlamEstimator.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_slam_estimator");
  ros::NodeHandle nh;
  PoseSlamEstimator pose_slam_estimator(&nh);
  return 0;
}
