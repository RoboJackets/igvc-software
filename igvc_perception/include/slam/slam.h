#ifndef SRC_SLAM_H
#define SRC_SLAM_H

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
// Pose2 == (Point3, Rot3)
#include <gtsam/geometry/Pose3.h>
// PriorFactor == Initial Pose
#include <gtsam/slam/PriorFactor.h>
// BetweenFactor == Odom measurement
#include <gtsam/slam/BetweenFactor.h>
// The factor graph we are creating. Nonlinear since angle measurements are nonlinear.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h>
// Helps initialize initial guess
#include <gtsam/nonlinear/Values.h>
// Custom Mag Factor based around Pose3
#include <slam/MagPoseFactor.h>
#include <gtsam/geometry/Rot3.h>
#include <tf/transform_listener.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/MagFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <slam/type_conversions.h>
#include <sensor_msgs/MagneticField.h>
#include <tf2/convert.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/message_filter.h>
#include <tf2/LinearMath/Transform.h>
// Convert between geodetic coordinates to local cartesian coordinates
#include <GeographicLib/Config.h>
#include <GeographicLib/LocalCartesian.hpp>

// Uncomment to enable debugging and have additional print statements
//#define _DEBUG 1

class Slam
{
public:
  Slam();

private:
  ros::NodeHandle pnh_;

  ros::Subscriber odom_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber gps_sub_;
  ros::Subscriber mag_sub_;
  ros::Subscriber wheel_sub_;

  ros::Publisher location_pub_;
  ros::Publisher gps_location_pub_;

  void gpsCallback(const sensor_msgs::NavSatFixConstPtr& gps);
  void imuCallback(const sensor_msgs::Imu &msg);
  void magCallback(const sensor_msgs::MagneticField &msg);
  void integrateAndAddIMUFactor();
  void wheelOdomCallback(const nav_msgs::Odometry &msg);
  void addWheelOdomFactor();
  void optimize();
  void initializeImuParams();
  void initializePriors();
  void initializeNoiseMatrices();
  void initializeDirectionOfLocalMagField();
  void addMagFactor();
  void updateTransform(const nav_msgs::Odometry &pos);
  static nav_msgs::Odometry createOdomMsg(const gtsam::Pose3 &pos, const gtsam::Vector3 &vel, const gtsam::Vector3 &ang);

  // Defining some types
  typedef gtsam::noiseModel::Diagonal noiseDiagonal;
  typedef gtsam::Vector3 Vec3;

  // Defining some variables
  gtsam::Values init_estimate_, result_;
#if defined(_DEBUG)
  gtsam::Values history_;
#endif

  gtsam::NonlinearFactorGraph graph_;
  tf2_ros::TransformBroadcaster world_transform_broadcaster_;
  unsigned long curr_index_;
  noiseDiagonal::shared_ptr gps_noise_, bias_noise_, mag_noise_, odometryNoise;
  gtsam::Unit3 local_mag_field_;
  gtsam::Point3 curr_mag_reading_;
  gtsam::Pose2 curr_wheelOdom_reading_;
  gtsam::ISAM2 isam_;
  gtsam::PreintegratedImuMeasurements accum_;
  ros::Time last_imu_measurement_;
  bool imu_connected_, imu_update_available_, firstReading;
  double scale_;
//  const GeographicLib::Geocentric& kWGS84;
  GeographicLib::LocalCartesian origin_ENU;

  const double KGRAVITY = 9.81;
};

#endif  // SRC_SLAM_H
