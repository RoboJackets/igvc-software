#ifndef StateEstimator_H_
#define StateEstimator_H_


// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>

// GTSAM includes
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/base/timing.h>
#include <GeographicLib/LocalCartesian.hpp>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include "ThreadedQueue.hpp"


class StateEstimator
{
private:

  ros::NodeHandle nh_;
  ros::Subscriber gpsSub_, imuSub_;
  ros::Publisher posePub_;

  ThreadedQueue<sensor_msgs::NavSatFixConstPtr> gpsQ_;
  ThreadedQueue<sensor_msgs::ImuConstPtr> imuQ_;

  double lastIMUT_;
  double accelBiasSigma_, gyroBiasSigma_;
  double gpsSigma_;

  GeographicLib::LocalCartesian enu_; /// Object to put lat/lon coordinates into local cartesian
  gtsam::Pose3 imuToGps_;

  // convenience function for taking a message pointer and returning the time stamp as a double
  template<class T>
  inline double ROS_TIME(T msg) { return msg->header.stamp.toSec(); }

public:
  StateEstimator();
  ~StateEstimator();
  void gpsCallback(sensor_msgs::NavSatFixConstPtr fix);
  void imuCallback(sensor_msgs::ImuConstPtr imu);
  void optimizationLoop();
};




#endif
