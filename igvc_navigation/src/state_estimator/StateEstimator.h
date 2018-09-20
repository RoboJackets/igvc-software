#ifndef StateEstimator_H_
#define StateEstimator_H_

// ROS includes
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

// GTSAM includes
#include <gtsam/base/timing.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <GeographicLib/LocalCartesian.hpp>

#include "ThreadedQueue.hpp"

class StateEstimator
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber gpsSub_, imuSub_;
  ros::Publisher posePub_;

  ThreadedQueue<sensor_msgs::NavSatFixConstPtr> gpsQ_;
  ThreadedQueue<sensor_msgs::ImuConstPtr> imuQ_;

  double lastImuT_;
  std::list<sensor_msgs::ImuConstPtr> imuMeasurements_;

  // parameters
  double accelSigma_, gyroSigma_, imuIntSigma_, accelBSigma_, gyroBSigma_, gravityMagnitude_;
  double accelBiasSigma_, gyroBiasSigma_;
  double gpsSigma_, yawSigma_;
  double optLag_;
  double imuFreq_;
  double priorOSigma_, priorPSigma_, priorVSigma_, priorABias_, priorGBias_;
  double gpsTSigma_;

  gtsam::PreintegratedImuMeasurements imuIntegrator_;

  GeographicLib::LocalCartesian enu_;  /// Object to put lat/lon coordinates into local cartesian
  gtsam::Pose3 imuToGps_;

  // variables for passing the state between threads
  mutable std::mutex mutex_;
  bool doneFirstOpt_;
  gtsam::Pose3 prevPose_;
  gtsam::Vector3 prevVel_;
  gtsam::imuBias::ConstantBias prevBias_;
  double currentTime_;

  // convenience function for taking a message pointer and returning the time stamp as a double
  template <class T>
  inline double ROS_TIME(T msg)
  {
    return msg->header.stamp.toSec();
  }

public:
  StateEstimator();
  ~StateEstimator();
  void gpsCallback(sensor_msgs::NavSatFixConstPtr fix);
  void imuCallback(sensor_msgs::ImuConstPtr imu);
  void optimizationLoop();
};

#endif
