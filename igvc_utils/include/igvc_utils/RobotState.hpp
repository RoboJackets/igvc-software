//
// Created by oswinso on 11/11/18.
//

#ifndef ROBOTSTATE_H
#define ROBOTSTATE_H

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

class RobotState
{
public:
    double x, y, roll, pitch, yaw;
    RobotState()
    {
      x = 0;
      y = 0;
      roll = 0;
      pitch = 0;
      yaw = 0;
    }

    RobotState(nav_msgs::Odometry::ConstPtr msg) {
      setState(msg);
    }

    void setState(nav_msgs::Odometry::ConstPtr msg)
    {
      x = msg->pose.pose.position.x;
      y = msg->pose.pose.position.y;
      tf::Quaternion quat;
      tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    }
};

#endif //ROBOTSTATE_H
