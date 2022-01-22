#include <parameter_assertions/assertions.h>

#include "swerve_odometer.h"


/**
 * Converts wheel velocities to odometry message using trigonometry for calculations
 * In the ros coordinate convention x is forward, y is leftward, and z is upward relative to the robot
 * The position is published in an absolute reference frame relative to the initial position
 * The velocities (twist) is in a reference frame relative to the robot
 */
SwerveOdometer::SwerveOdometer() : pNh{"~"}
{
  if (!getParams())
  {
    ROS_ERROR_STREAM("Unable to import parameters!");
    return;
  }

  sub = nh.subscribe("/encoders", 10, &SwerveOdometer::enc_callback, this);
  pub = nh.advertise<nav_msgs::Odometry>("/wheel_odometry", 10);

  // initializing sequence number for messages
  seq = 0;

  // initialize position - map published is relative to position at time t=0
  x = 0;
  y = 0;
  yaw = 0;
}

void SwerveOdometer::enc_callback(const igvc_msgs::velocity_quad msg)
{
  wheel_info[0][0] = msg.fl_velocity;
  wheel_info[1][0] = msg.bl_velocity;
  wheel_info[2][0] = msg.br_velocity;
  wheel_info[3][0] = msg.fr_velocity;

  wheel_info[0][1] = msg.fl_angle;
  wheel_info[1][1] = msg.bl_angle;
  wheel_info[2][1] = msg.br_angle;
  wheel_info[3][1] = msg.fr_angle;

  double deltaT = msg.duration;

  for (int i = 0; i < num_wheels; ++i) {
    if (wheel_info[i][0] < 0) {
      wheel_info[i][0] = fabs(wheel_info[i][0]);
      wheel_info[i][1] = theta_map(wheel_info[i][1] + M_PI);
    }
  }

  std::vector<std::array<double, 2>> intersections;
  for (int i = 0; i < num_wheels - 1; ++i) {
    double theta_i = theta_map(wheel_info[i][1] + M_PI_2);
    double tan_i = tan(theta_i);
    double b_i = positions_list[i][1] - tan_i * positions_list[i][0];

    for (int j = i + 1; j < num_wheels; ++j) {
      double theta_j = theta_map(wheel_info[j][1] + M_PI_2);
      double tan_j = tan(theta_j);
      double b_j = positions_list[j][1] - tan_j * positions_list[j][0];

      if (isclose(theta_i, theta_j) || isclose(theta_i, theta_j, 0.00001, 2*M_PI)) {
        intersections.push_back({INFINITY, INFINITY});
      }
      else if (isclose(theta_i, theta_j, 0.00001, M_PI) || isclose(theta_i, theta_j, 0.00001, -M_PI)){
        intersections.push_back({ (positions_list[i][0]+positions_list[j][0] / 2) ,
          (positions_list[i][1]+positions_list[j][1] / 2)});
      }
      else {
        double val1 = fabs(( (b_i - b_j) / (tan_j - tan_i) ));
        double val2 = fabs(( (tan_j*b_i - tan_i*b_j) / (tan_j - tan_i) ));

        if ( val1 > inf_tol || val2 > inf_tol) {
          intersections.push_back({INFINITY, INFINITY});
        } else {
          intersections.push_back({val1, val2});
        }
      }
    }
  }

  double l_x = 0, l_x_vh = 0, l_y = 0, l_y_vh = 0, w = 0;

  std::array<double,2> average_intersection = {0,0} ;

  bool inf_all = true;
  for (const auto& val : intersections) {
    if (!std::isinf(val[0]) && !std::isinf(val[1])) {
      inf_all = false;
      break;
    }
  }

  if (inf_all) {
    for (int i = 0; i < num_wheels; ++i)
    {
      l_x_vh += ( wheel_info[i][0] * wheel_radius * cos(wheel_info[i][1]) ) / num_wheels;
      l_y_vh += ( wheel_info[i][0] * wheel_radius * sin(wheel_info[i][1]) ) / num_wheels;
    }
  } else {
    size_t n = intersections.size();

    for (size_t i = 0; i < n; ++i) {
      if ((fabs(intersections[i][0]-intersections[i-1][0])>intersection_tol_
        || fabs(intersections[i][1]-intersections[i-1][1])>intersection_tol_)
        && i != 0)
      {
        ROS_ERROR_STREAM("intersections are not close enough to get an average, dropping!");

        return;
      }
      else
      {
          average_intersection[0] += intersections[i][0] / n;
          average_intersection[1] += intersections[i][1] / n;
      }
    }
    for (int i = 0; i < num_wheels; ++i) {
      //ignore the wheel if the intersection is on its center of rotation
      if (isclose(average_intersection[0], wheel_info[i][0]) && isclose(average_intersection[1], wheel_info[i][1]))
      {
        continue;
      }
      auto icr_wh = std::array<double,2>{wheel_info[i][0] - average_intersection[0] , wheel_info[i][1] - average_intersection[1]};
      if (isinf(icr_wh[0]) || isinf(icr_wh[1]))
          ROS_WARN_STREAM("icr_wh is inf");
      if (isclose(icr_wh[0], 0) || isclose(icr_wh[1], 0))
          ROS_WARN_STREAM("icr_wh for wheel "<< i <<" is zero. icr is just over it!");

      w += (wheel_info[i][0] * wheel_radius * sin(wheel_info[i][1])) / (2*icr_wh[0]) - (wheel_info[i][0] * wheel_radius * cos(wheel_info[i][1]))/(2*icr_wh[1]);
    }
    w /= num_wheels; 
    l_x_vh =      average_intersection[1] * w;
    l_y_vh = -1 * average_intersection[0] * w;
  }

  if (isnan(l_x_vh) || isnan(l_y_vh) || isnan(w)) {
    ROS_ERROR_STREAM("estimated vx,vy or wz is nan");
    return;
  }

  if (isinf(l_x_vh) || isinf(l_y_vh) || isinf(w)) {
    ROS_ERROR_STREAM("estimated vx,vy or wz is inf");
    return;
  }

  l_x = l_x_vh * cos(yaw) - l_y_vh * sin(yaw);
  l_y = l_x_vh * sin(yaw) + l_y_vh * cos(yaw);

  integrateRungeKutta2(linear_x, linear_y, angular, deltaT);

  // rolling filter to remove noise
  linear_x = (alpha * l_x) + (1.0 - alpha) * linear_x;
  linear_y = (alpha * l_y) + (1.0 - alpha) * linear_y;
  angular = (alpha * w) + (1.0 - alpha) * angular;

  geometry_msgs::Vector3 linearVelocities;
  linearVelocities.z = 0;
  linearVelocities.x = linear_x;
  linearVelocities.y = linear_y;


  // set angular velocities - assuming 2D operation
  geometry_msgs::Vector3 angularVelocities;
  angularVelocities.x = 0.0;
  angularVelocities.y = 0.0;
  angularVelocities.z = angular;

  nav_msgs::Odometry odom;
  odom.twist.twist.linear = linearVelocities;
  odom.twist.twist.angular = angularVelocities;


  // enter message info for global position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

  // Row-major representation of the 6x6 covariance matrix
  // The orientation parameters use a fixed-axis representation.
  // In order, the parameters are:
  // (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
  odom.twist.covariance = { 0.02, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 0.25, 1e-4, 1e-4, 1e-4, 1e-4,
                            1e-4, 1e-4, 1e6,  1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e6,  1e-4, 1e-4,
                            1e-4, 1e-4, 1e-4, 1e-4, 1e6,  1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 0.62 };
  // the position covariance takes same form as twist covariance above
  // this grows without bounds as error accumulates - disregard exact reading with high variance
  odom.pose.covariance = { 0.01, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 0.01, 1e-6, 1e-6, 1e-6, 1e-6,
                           1e-6, 1e-6, 1e6,  1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e6,  1e-6, 1e-6,
                           1e-6, 1e-6, 1e-6, 1e-6, 1e6,  1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e6 };

  // setting sequence of message
  odom.header.seq = seq++;

  // setting reference frames
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";

  // set time then publish
  odom.header.stamp = ros::Time::now();
  pub.publish(odom);
}

bool SwerveOdometer::getParams()
{
  assertions::param(pNh, "wheel_radius", wheel_radius, 0.1375);
  assertions::param(pNh, "inf_tol", inf_tol, 1000.0);
  assertions::param(pNh, "intersection_tol_", intersection_tol_, 0.1);
  assertions::param(pNh, "alpha", alpha, 0.1);

  XmlRpc::XmlRpcValue xml_list;

  // positions
  if (!pNh.getParam("swerve_drive/positions", xml_list))
  {
    ROS_ERROR_STREAM("Unable to retrieve position list");
    return false;
  }
  if (xml_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR_STREAM("position list not of type array");
    return false;
  }
  if (xml_list.size() != num_wheels)
  {
    ROS_ERROR_STREAM("position list not of size 4");
    return false;
  }
  for (int i = 0; i < num_wheels; ++i)
  {
    if (xml_list[i].getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR_STREAM("position #" << i << " is not of type array");
      return false;
    }
    if (xml_list[i].size() != 2)
    {
      ROS_ERROR_STREAM("position #" << i << " is not size 2");
    }
    std::array<double, 2> hold;
    for (int j = 0; j < 2; ++j)
    {
      if (xml_list[i][j].getType() != XmlRpc::XmlRpcValue::TypeDouble &&
          xml_list[i][j].getType() != XmlRpc::XmlRpcValue::TypeInt)
      {
        ROS_ERROR_STREAM("position #{" << i << ", " << j << "} is not of type double or int");
        return false;
      }
      hold[j] = static_cast<double>(xml_list[i][j]);
    }
    positions_list[i] = hold;
  }
  return true;
}

void SwerveOdometer::integrateRungeKutta2(double linear_x, double linear_y, double angular, const double dt)
{
  const double direction = yaw + angular;

  /// Runge-Kutta 2nd order integration:
  x       += ( linear_x * cos(direction) - linear_y * sin(direction) ) * dt;
  y       += ( linear_x * sin(direction) + linear_y * cos(direction) ) * dt;
  yaw     += angular * dt;
}

double SwerveOdometer::theta_map(const double& theta)
{
  if (isclose(theta, M_PI))
  {
    return M_PI;  // to keep the 180 as is
  }
  return fmod((theta + M_PI), 2 * M_PI) - M_PI;
}

double SwerveOdometer::isclose(const double& a, const double& b, const double tol, const double bias)
{
  return fabs(bias - fabs(a - b)) <= tol;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wheel_odom");

  SwerveOdometer odom;

  ROS_INFO_STREAM("swerve wheel odometry node has started");

  ros::spin();
}