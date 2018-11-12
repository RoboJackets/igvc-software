#ifndef ROBOTSTATE_H
#define ROBOTSTATE_H

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>

class RobotState
{
public:
    double x{0};
    double y{0};
    double roll{0};
    double pitch{0};
    double yaw{0};

    friend std::ostream &operator<<(std::ostream &out, const RobotState &state);

    RobotState()
    = default;

    RobotState(nav_msgs::Odometry::ConstPtr msg)
    {
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

    Eigen::Vector3d getVector3d()
    {
      return Eigen::Vector3d(x, y, yaw);
    }
};

std::ostream &operator<<(std::ostream &out, const RobotState &state)
{
  out << "(" << state.x << ", " << state.y << ", " << state.yaw << ")";
  return out;
}

#endif //ROBOTSTATE_H
