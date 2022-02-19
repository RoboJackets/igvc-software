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

  prev_time = ros::Time::now();
  sub = nh.subscribe("/encoders", 10, &SwerveOdometer::enc_callback, this);
  subAng = nh.subscribe("/ground_truth", 10, &SwerveOdometer::ang_callback, this);
  pub = nh.advertise<nav_msgs::Odometry>("/wheel_odometry", 10);
  intersectionPub = nh.advertise<geometry_msgs::PointStamped>("/wheel_odometry_intersection", 10);
  intersectionPubAvg = nh.advertise<geometry_msgs::PointStamped>("/wheel_odometry_intersection_avg", 10);

  // initializing sequence number for messages
  seq = 0;

  // initialize position - map published is relative to position at time t=0
  x = 0;
  y = 0;
  yaw = 0;
}

void SwerveOdometer::ang_callback(const nav_msgs::Odometry msg)
{
  actual_angular_ = msg.twist.twist.angular.z;
}

void SwerveOdometer::enc_callback(const igvc_msgs::velocity_quad msg)
{
  wheel_info[0][0] = msg.fl_velocity / wheel_radius;
  wheel_info[1][0] = msg.bl_velocity / wheel_radius;
  wheel_info[2][0] = msg.br_velocity / wheel_radius;
  wheel_info[3][0] = msg.fr_velocity / wheel_radius;

  wheel_info[0][1] = msg.fl_angle;
  wheel_info[1][1] = msg.bl_angle;
  wheel_info[2][1] = msg.br_angle;
  wheel_info[3][1] = msg.fr_angle;

  for (auto& wheel : wheel_info) {
    if (isclose(wheel[0], 0, .05)) {
      wheel[0] = 0;
    }
    if (isclose(wheel[1], 0, .05)) {
      wheel[1] = 0;
    }
  }

  ros::Time curr_time = ros::Time::now();
  ros::Duration duration = curr_time - prev_time;

  double deltaT;
  if (duration.toSec() > msg.duration) {
    deltaT = duration.toSec();
  } else {
    deltaT = msg.duration;
  }
  deltaT = msg.duration;
  // ROS_INFO_STREAM("deltaT: " << deltaT);

  for (int i = 0; i < num_wheels; ++i) {
    if (wheel_info[i][0] < 0) {
      wheel_info[i][0] = fabs(wheel_info[i][0]);
      wheel_info[i][1] = theta_map(wheel_info[i][1] + M_PI);
    }
  }

  //intersection point 
  double theta1,m1,b1,theta,m,b;
  std::vector<std::array<double,2>> intersections;
  for (size_t j=0; j<num_wheels-1; ++j)
  {
    theta1 = theta_map(wheel_info[j][1]+M_PI_2);
    // Slope
    m1 = tan(theta1);
    b1 = positions_list[j][1]- m1 * positions_list[j][0];
    for (size_t i=j+1; i<num_wheels; ++i)
    {
      theta = theta_map(wheel_info[i][1]+M_PI_2);
      // Slope
      m = tan(theta);

      // h = (m2x2 - m1x1)- (y2-y1) / (m2-m1);

      b = positions_list[i][1]- m * positions_list[i][0];

      if ( isclose(theta1,theta) || isclose(theta1,theta,0.01,2*M_PI) ) 
      {
        intersections.push_back({INFINITY,INFINITY});
      }
      else if ( isclose(theta1,theta,0.01,M_PI) || isclose(theta1,theta,0.01,-M_PI) ) 
      {
        intersections.push_back({(positions_list[j][0]+positions_list[i][0])/2 , (positions_list[j][1]+positions_list[i][1])/2});
        geometry_msgs::PointStamped intersectionMsg;
        intersectionMsg.header.frame_id = "base_link";
        intersectionMsg.header.stamp = ros::Time::now();
        intersectionMsg.point.x = (positions_list[j][0]+positions_list[i][0])/2;
        intersectionMsg.point.y = (positions_list[j][1]+positions_list[i][1])/2;
        intersectionPub.publish(intersectionMsg);
        
      }
      else
      {
        if (m == 0 && m1 == 0) {
          intersections.push_back({INFINITY,INFINITY});
          continue;
        }
        // double h = ((m*positions_list[i][0] - m1*positions_list[j][0]) - (positions_list[i][1] - positions_list[j][1])) / (m - m1);
        // double k = m1 * (h - positions_list[j][0]) + positions_list[j][1];

        double h = ((b1-b)/(m-m1));
        double k = ((m*b1-m1*b)/(m-m1));

        if (fabs(h)>inf_tol || fabs(k)>inf_tol )
        {
          intersections.push_back({INFINITY,INFINITY});
        }
        else
        {
          intersections.push_back({ h , k });
          // ROS_INFO_STREAM("H: " << ((b1-b)/(m-m1)) << ", K: " << ((m*b1-m1*b)/(m-m1)));
          // ROS_INFO_STREAM("H_new: " << h << ", K_new: " << k);
          geometry_msgs::PointStamped intersectionMsg;
          intersectionMsg.header.frame_id = "base_link";
          intersectionMsg.header.stamp = ros::Time::now();
          intersectionMsg.point.x = h;
          intersectionMsg.point.y = k;
          intersectionPub.publish(intersectionMsg);
        }
      }
    }
  }

  double linear_x = 0,linear_x_vh = 0,linear_y = 0,linear_y_vh = 0,angular = 0;
  std::array<double,2> average_intersection = {0,0} ;

  //detecting if all or some of the intersections is inf
  bool inf_all = true;
  for (const auto& it: intersections)
  {
    if (! (std::isinf(it[0]) || std::isinf(it[1])))
    {
      inf_all = false;
      break;
    }
  }

  if (inf_all)
  {
    ROS_INFO_STREAM("inf_all");
    angular=0;
    for (int i=0; i<num_wheels; ++i)
    {
      linear_x_vh += ( wheel_info[i][0]*wheel_radius*cos(wheel_info[i][1]) ) / num_wheels;
      linear_y_vh += ( wheel_info[i][0]*wheel_radius*sin(wheel_info[i][1]) ) / num_wheels;
      ROS_INFO_STREAM("linear_x_vh: " << linear_x_vh);
      ROS_INFO_STREAM("linear_y_vh: " << linear_y_vh);
    }
  }
  else
  {
    double furthest_dist = 0;
    for (size_t i=1; i<intersections.size(); ++i)
    {
      if((fabs(intersections[i][0]-intersections[i-1][0])>intersection_tol_ || fabs(intersections[i][1]-intersections[i-1][1])>intersection_tol_))
      {
        ROS_ERROR_STREAM("intersections are not close enough to get an average, dropping!");
        for (size_t i=0; i<num_wheels; ++i)
        {
          ROS_WARN_STREAM("theta, omega: "<<wheel_info[i][1]<<" "<<wheel_info[i][0]);
        }
        for (size_t i=0; i<intersections.size(); ++i)
        {
          ROS_WARN_STREAM("intersection i:"<<i<<" , "<<intersections[i][0]<<"  "<<intersections[i][1]);
        }
        return;
      }
      else
      {
        double dist = std::hypot(intersections[i][0], intersections[i][1]);
        if (dist > furthest_dist) {
          average_intersection[0] = intersections[i][0];
          average_intersection[1] = intersections[i][1];
          furthest_dist = dist;
        }
          // average_intersection[0] += intersections[i][0];
          // average_intersection[1] += intersections[i][1];
      }
    }
    // average_intersection[0] /= intersections.size();
    // average_intersection[1] /= intersections.size();

    geometry_msgs::PointStamped intersectionMsg;
    intersectionMsg.header.frame_id = "base_link";
    intersectionMsg.header.stamp = ros::Time::now();
    intersectionMsg.point.x = average_intersection[0];
    intersectionMsg.point.y = average_intersection[1];
    intersectionPubAvg.publish(intersectionMsg);
    
    // ROS_INFO_STREAM("average intersection: "<<average_intersection[0]<<" "<<average_intersection[1]);
    for (int i=0; i<num_wheels; ++i)
    {
      //ignore the wheel if the intersection is on its center of rotation
      if (isclose(average_intersection[0],positions_list[i][0])&&isclose(average_intersection[1],positions_list[i][1]))
      {
        continue;
      }
      auto icr_wh = std::array<double,2>{positions_list[i][0]-average_intersection[0] , positions_list[i][1]-average_intersection[1]};
      // auto icr_wh = std::array<double,2>{fabs(positions_list[i][0]-average_intersection[0]) , fabs(positions_list[i][1]-average_intersection[1])};
      if (isinf(icr_wh[0])||isinf(icr_wh[1]))
          ROS_WARN_STREAM("icr_wh is inf");
      if (isclose(icr_wh[0],0)||isclose(icr_wh[1],0))
          ROS_WARN_STREAM("icr_wh for wheel "<< i <<" is zero. icr is just over it!");
      
      double rad = std::hypot(icr_wh[0], icr_wh[1]);
      double ang_computed = wheel_radius * wheel_info[i][0] / rad;
      // angular += ang_computed;

      double ang_x = (wheel_info[i][0]*wheel_radius*sin(wheel_info[i][1]))/(icr_wh[0]);
      double ang_y = (wheel_info[i][0]*wheel_radius*cos(wheel_info[i][1]))/(icr_wh[1]);
      double ang_computed1 = ang_x/2 - ang_y/2;

      ROS_INFO_STREAM("angular: " << ang_computed);
      ROS_INFO_STREAM("angular1: " << ang_computed1);

      angular += ang_computed1;

      if (isinf(angular)) ROS_WARN_STREAM("angular is the problem");
    }
    angular/=num_wheels; 
    // ROS_INFO_STREAM("angular: " << angular);
    linear_x_vh =      average_intersection[1] * angular;
    linear_y_vh = -1 * average_intersection[0] * angular;

    // linear_x_vh =      average_intersection[1] * actual_angular_;
    // linear_y_vh = -1 * average_intersection[0] * actual_angular_;
  }

  if(isnan(linear_x_vh)||isnan(linear_y_vh)||isnan(angular))
  {
    ROS_ERROR_STREAM("estimated vx,vy or wz is nan");
    for (const auto& it: wheel_info)
    {
      ROS_INFO_STREAM("theta "<<it[1]);
    }
    for (const auto& it: wheel_info)
    {
      ROS_INFO_STREAM("omega "<<it[0]);
    }
    for (const auto& it: intersections)
    {
      ROS_INFO_STREAM("intersection "<<it[0]<<" "<<it[1]);
    }
    ROS_INFO_STREAM("average intersection: "<<average_intersection[0]<<" "<<average_intersection[1]);
    ROS_INFO_STREAM("linearx "<<linear_x_vh<<" lineary "<<linear_y_vh<<" angular "<<angular);
    return;
  }
  if(isinf(linear_x_vh)||isinf(linear_y_vh)||isinf(angular))
  {
    ROS_ERROR_STREAM("estimated vx,vy or wz is inf");
    for (const auto& it: wheel_info)
    {
      ROS_INFO_STREAM("theta "<<it[1]);
    }
    for (const auto& it: wheel_info)
    {
      ROS_INFO_STREAM("omega "<<it[0]);
    }
    for (const auto& it: intersections)
    {
      ROS_INFO_STREAM("intersection "<<it[0]<<" "<<it[1]);
    }
    ROS_INFO_STREAM("average intersection: "<<average_intersection[0]<<" "<<average_intersection[1]);
    ROS_INFO_STREAM("linearx "<<linear_x_vh<<" lineary "<<linear_y_vh<<" angular "<<angular);
    return;
  }

  // ROS_INFO_STREAM("linearx "<<linear_x_vh<<" lineary "<<linear_y_vh<<" angular "<<angular);

  linear_x = linear_x_vh * cos(yaw) - linear_y_vh * sin(yaw);
  linear_y = linear_x_vh * sin(yaw) + linear_y_vh * cos(yaw);


  /// Integrate odometry:
  integrateRungeKutta2(linear_x, linear_y, angular_, deltaT);
  // integrateRungeKutta2(linear_x, linear_y, actual_angular_, deltaT);


  /// Estimate speeds using a rolling mean to filter them out:
  // linear_acc_x_ (linear_x);
  // linear_acc_y_ (linear_y);
  // angular_acc_  (angular);

  // linear_x_ = bacc::rolling_mean(linear_acc_x_);
  // linear_y_ = bacc::rolling_mean(linear_acc_y_);
  // angular_ = bacc::rolling_mean(angular_acc_);

  linear_x_ = (alpha * linear_x) + (1.0 - alpha) * linear_x_;
  linear_y_ = (alpha * linear_y) + (1.0 - alpha) * linear_y_;
  angular_ = (alpha * angular) + (1.0 - alpha) * angular_;
  // angular_ = actual_angular_;
  
  // ROS_INFO_STREAM(std::endl);
  // return true;

  geometry_msgs::Vector3 linearVelocities;
  linearVelocities.z = 0;
  linearVelocities.x = linear_x_;
  linearVelocities.y = linear_y_;


  // set angular velocities - assuming 2D operation
  geometry_msgs::Vector3 angularVelocities;
  angularVelocities.x = 0.0;
  angularVelocities.y = 0.0;
  angularVelocities.z = angular_;

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
  prev_time = curr_time;
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
  /// Runge-Kutta 2nd order integration:
  yaw     += angular * dt;
  x       += linear_x * dt;
  y       += linear_y * dt;
  // ROS_INFO_STREAM("x  : " << x);
  // ROS_INFO_STREAM("y  : " << y);
  // ROS_INFO_STREAM("dt: " << dt);
  // ROS_INFO_STREAM("angular: " << angular);
  // ROS_INFO_STREAM("yaw: " << yaw);
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