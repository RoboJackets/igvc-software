#ifndef SWERVE_ODOMETER_H
#define SWERVE_ODOMETER_H
#include <igvc_msgs/velocity_quad.h>
#include <ros/ros.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class SwerveOdometer
{
public:
  SwerveOdometer();

private:
  // ros infastructure
  ros::NodeHandle nh;
  ros::NodeHandle pNh;

  ros::Publisher pub;
  ros::Subscriber sub;
  tf::TransformBroadcaster odom_broadcaster;
  int seq;

  // robot constant
  double wheel_sep;
  std::array<std::array<double, 2>, 4> positions_list;
  std::array<std::array<double, 2>, 4> wheel_info;
  const int num_wheels = 4;
  double wheel_radius;

  // Current pose:
  double x;        //   [m]
  double y;        //   [m]
  double yaw;      // [rad]

  // Current velocity:
  double linear_x;  //   [m/s]
  double linear_y;  //   [m/s]
  double angular;   // [rad/s]

  double inf_tol;
  double intersection_tol_;
  double alpha;

  // callback for encoder subscriber
  void enc_callback(const igvc_msgs::velocity_quad msg);
  bool getParams();
  void integrateRungeKutta2(double linear_x, double linear_y, double angular, const double dt);
  double theta_map(const double& theta);
  double isclose(const double& a, const double& b, const double tol = 0.00001, const double bias = 0);

};
#endif
