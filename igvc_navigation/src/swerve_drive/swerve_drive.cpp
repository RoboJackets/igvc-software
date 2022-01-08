#include <parameter_assertions/assertions.h>
#include <ros/ros.h>

#include "swerve_drive.h"

SwerveDrive::SwerveDrive() : pNh{ "~" }
{
  if (!getParams()) {
    ROS_ERROR_STREAM("Unable to import parameters!");
    return;
  }
  assertions::param(pNh, "max_vel", max_vel_, 3.0);
  mbf_twist_ = nh.subscribe("/cmd_vel", 1, &SwerveDrive::twistToVelocity, this);
  vel_pub_ = nh.advertise<igvc_msgs::velocity_quad>("/motors", 1);
}

void SwerveDrive::twistToVelocity(geometry_msgs::Twist twist)
{
  if (!std::isfinite(twist.linear.x) || !std::isfinite(twist.linear.y) || !std::isfinite(twist.angular.z)) {
    ROS_WARN_THROTTLE(1.0, "Recieved NaN in velocity command. Ignoring.");
    return;
  }

  const double speedX = twist.linear.x;
  const double speedY = twist.linear.y;
  const double rotation = twist.angular.z;

  const size_t num_wheels = 4;
  for (size_t i = 0; i < num_wheels; ++i) {
    double wheel_vx = speedX - rotation * positions_list[i][1];
    double wheel_vy = speedY + rotation * positions_list[i][0];

    double wheel_V = sqrt(pow(wheel_vx, 2) + pow(wheel_vy, 2)) / radii_list[i];
    double wheel_dir = atan2(wheel_vy, wheel_vx);

    int omega_dir = set_command_angle(wheel_dir, i);
    if (omega_dir == 0) {
      ROS_ERROR_STREAM("Unable to set steering angle! Resetting to default");
      // igvc_msgs::velocity_quad vel_msg;
      // vel_pub_.publish(vel_msg);
      // for (size_t j = 0; j < 4; ++j) {
      //   wheel_info[j][1] = 0;
      //   wheel_info[j][0] = 0;
      // }
      return;
    }

    wheel_V = std::min(max_vel_, wheel_V);

    wheel_info[i][0] = wheel_V * omega_dir;

    // wheel_info[i][0] = wheel_V;
    // wheel_info[i][1] = wheel_dir;
  }
  // contructing vel message
  igvc_msgs::velocity_quad vel_msg;
  vel_msg.fl_velocity = wheel_info[0][0];
  vel_msg.bl_velocity = wheel_info[1][0];
  vel_msg.br_velocity = wheel_info[2][0];
  vel_msg.fr_velocity = wheel_info[3][0];
  
  vel_msg.fl_angle = wheel_info[0][1];
  vel_msg.bl_angle = wheel_info[1][1];
  vel_msg.br_angle = wheel_info[2][1];
  vel_msg.fr_angle = wheel_info[3][1];
  vel_msg.duration = 0.02;
  vel_msg.header.stamp = ros::Time::now();
  
  vel_pub_.publish(vel_msg);
}

int SwerveDrive::set_command_angle(const double& target, const int wheel_idx)
{
    bool triple_point = false;
    if (utils::isclose(target,0) || utils::isclose(target,M_PI) || utils::isclose(target,-M_PI))
    {
        triple_point = true;        
    }
    auto supplementary = utils::theta_map(target+M_PI);        
    double supplementary2 = 0;
    if (triple_point)
    {
        supplementary2 = utils::isclose(supplementary,M_PI) ? -M_PI : M_PI;
    }

    // if (target > 1.58 || target < -1.58) {
    //   ROS_WARN_STREAM("bro wtf is this target?: " << target);
    // }
    // if (supplementary > 1.58 || supplementary < -1.58) {
    //   ROS_WARN_STREAM("bro wtf is this supplementary?: " << supplementary);
    // }
    // if (supplementary2 > 1.58 || supplementary2 < -1.58) {
    //   ROS_WARN_STREAM("bro wtf is this supplementary2?: " << supplementary2);
    // }
    ROS_INFO_STREAM("target: " << target);
    ROS_INFO_STREAM("supplementary: " << supplementary);
    ROS_INFO_STREAM("supplementary2: " << supplementary2);

    auto interval1  = interval(std::array<double,2>({wheel_info[wheel_idx][1],target}),"close" );
    auto interval2  = interval(std::array<double,2>({wheel_info[wheel_idx][1],supplementary}),"close" );
    auto interval3  = interval1.complement();
    auto interval4  = interval2.complement();

    // interval1.print();
    // ROS_INFO_STREAM("break");
    // interval2.print();
    // ROS_INFO_STREAM("break");
    // interval3.print();
    // ROS_INFO_STREAM("break");
    // interval4.print();
    
    interval interval5;
    interval interval6;
    if (triple_point)
    {
        interval5  = interval(std::array<double,2>({wheel_info[wheel_idx][1],supplementary2}),"close" );
        interval6  = interval5.complement();
    }
    std::vector<double> lengths = {interval1.len(), interval3.len(), interval2.len(), interval4.len()};

    int min_arg;
    std::vector<bool> intersects = {interval1.is_intersecting(limits_list[wheel_idx]), interval3.is_intersecting(limits_list[wheel_idx]),
      interval2.is_intersecting(limits_list[wheel_idx]), interval4.is_intersecting(limits_list[wheel_idx])};
    if (intersects[0] && intersects[1] && intersects[2] && intersects[3])
    {
        interval1.print();
        interval3.print();
        interval2.print();
        interval4.print();
        ROS_WARN_STREAM("bro wtf");
        //print something
        // return 0;
    }
    lengths = {lengths[0] + 2*M_PI*intersects[0], lengths[1] + 2*M_PI*intersects[1], lengths[2] + 2*M_PI*intersects[2], lengths[3] + 2*M_PI*intersects[3]}; //punished with intersection
    if (triple_point)
    {  
         lengths.push_back(interval5.len()+ 2*M_PI*interval5.is_intersecting(limits_list[wheel_idx]));
         lengths.push_back(interval6.len()+ 2*M_PI*interval6.is_intersecting(limits_list[wheel_idx]));
    }

    int omega_direc_;
    min_arg = std::distance(lengths.begin(), std::min_element(lengths.begin(), lengths.end()));        
    if (min_arg < 2)
    {
        omega_direc_ = 1;
        if (target > 1.58 || target < -1.58) {
          ROS_WARN_STREAM("bro wtf is this: " << target);
        }
        wheel_info[wheel_idx][1] = target;
        return omega_direc_;
    }
    if (min_arg < 4)
    {
        omega_direc_ = -1;
        if (supplementary > 1.58 || supplementary < -1.58) {
          ROS_WARN_STREAM("bro wtf is this: " << supplementary);
        }
        wheel_info[wheel_idx][1] = supplementary;
        return omega_direc_;
    }
    else
    {
        omega_direc_ = -1;
        if (supplementary2 > 1.58 || supplementary2 < -1.58) {
          ROS_WARN_STREAM("bro wtf is this: " << supplementary2);
        }
        wheel_info[wheel_idx][1] = supplementary2;
        return omega_direc_;
    }
}

bool SwerveDrive::getParams()
{
  XmlRpc::XmlRpcValue xml_list;
  // radii
  if (!nh.getParam("swerve_drive/radii", xml_list)) {
    ROS_ERROR_STREAM("Unable to retrieve radii list");
    return false;
  }
  if (xml_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR_STREAM("radii list not of type array");
    return false;
  }
  const int num_wheels = 4;
  if (xml_list.size() != num_wheels) {
    ROS_ERROR_STREAM("radii list not of size 4");
    return false;
  }
  for (int i = 0; i < num_wheels; ++i) {
    if (xml_list[i].getType() != XmlRpc::XmlRpcValue::TypeDouble &&
      xml_list[i].getType() != XmlRpc::XmlRpcValue::TypeInt) {
      ROS_ERROR_STREAM("radii #" << i << " is not of type double or int");
      return false;
    }
    radii_list[i] = static_cast<double>(xml_list[i]);
  }

  // positions
  if (!nh.getParam("swerve_drive/positions", xml_list)) {
    ROS_ERROR_STREAM("Unable to retrieve position list");
    return false;
  }
  if (xml_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR_STREAM("position list not of type array");
    return false;
  }
  if (xml_list.size() != num_wheels) {
    ROS_ERROR_STREAM("position list not of size 4");
    return false;
  }
  for (int i = 0; i < num_wheels; ++i) {
    if (xml_list[i].getType() != XmlRpc::XmlRpcValue::TypeArray) {
      ROS_ERROR_STREAM("position #" << i << " is not of type array");
      return false;
    }
    if (xml_list[i].size() != 2) {
      ROS_ERROR_STREAM("position #" << i << " is not size 2");
    }
    std::array<double,2> hold;
    for (int j = 0; j < 2; ++j) {
      if (xml_list[i][j].getType() != XmlRpc::XmlRpcValue::TypeDouble &&
        xml_list[i][j].getType() != XmlRpc::XmlRpcValue::TypeInt) {
        ROS_ERROR_STREAM("position #{" << i << ", " << j << "} is not of type double or int");
        return false;
      }
      hold[j] = static_cast<double>(xml_list[i][j]);
    }
    positions_list[i] = hold;
  }

  // limits
  if (!nh.getParam("swerve_drive/limits", xml_list)) {
    ROS_ERROR_STREAM("Unable to retrieve limits list");
    return false;
  }
  if (xml_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR_STREAM("limits list not of type array");
    return false;
  }
  if (xml_list.size() != num_wheels) {
    ROS_ERROR_STREAM("limits list not of size 4");
    return false;
  }
  for (int i = 0; i < num_wheels; ++i) {
    if (xml_list[i].getType() != XmlRpc::XmlRpcValue::TypeArray) {
      ROS_ERROR_STREAM("limits #" << i << " is not of type array");
      return false;
    }
    if (xml_list[i].size() != 2) {
      ROS_ERROR_STREAM("limits #" << i << " is not size 2");
    }
    std::array<double,2> hold;
    for (int j = 0; j < 2; ++j) {
      if (xml_list[i][j].getType() != XmlRpc::XmlRpcValue::TypeDouble &&
        xml_list[i][j].getType() != XmlRpc::XmlRpcValue::TypeInt) {
        ROS_ERROR_STREAM("limits #{" << i << ", " << j << "} is not of type double or int");
        return false;
      }
      hold[j] = static_cast<double>(xml_list[i][j]);
    }
    interval curr = interval(hold,"close");
    limits_list.push_back(curr.complement());
    // curr.print();
    // limits_list.back().print();
  }
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "swerve_drive");
  SwerveDrive swerve_drive;
  ros::spin();
}
