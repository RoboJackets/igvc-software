#ifndef ROBOTSTATE_H
#define ROBOTSTATE_H

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <igvc_utils/NodeUtils.hpp>
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

    explicit RobotState(const nav_msgs::Odometry::ConstPtr &msg)
    {
      setState(msg);
    }

    void setState(const nav_msgs::Odometry::ConstPtr &msg)
    {
      x = msg->pose.pose.position.x;
      y = msg->pose.pose.position.y;
      tf::Quaternion quaternion;
      tf::quaternionMsgToTF(msg->pose.pose.orientation, quaternion);
      tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    }

    Eigen::Vector3d getVector3d()
    {
      return {x, y, yaw};
    }

    /**
    returns the euclidian distance from the current robot position to a
    specified position [x2,y2]
    */
    double distTo(double x2, double y2) {
        return igvc::get_distance(this->x,this->y,x2,y2);
    }

    bool operator== (const RobotState& other)
    {
      return std::tie(x, y, roll, pitch, yaw) == std::tie(other.x, other.y, other.roll, other.pitch, other.yaw);
    }
};

std::ostream &operator<<(std::ostream &out, const RobotState &state)
{
  out << "(" << state.x << ", " << state.y << ", " << state.yaw << ")";
  return out;
}

#endif //ROBOTSTATE_H
