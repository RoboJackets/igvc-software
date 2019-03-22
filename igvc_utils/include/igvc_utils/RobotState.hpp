#ifndef ROBOTSTATE_H
#define ROBOTSTATE_H

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <igvc_utils/NodeUtils.hpp>
#include <igvc_msgs/velocity_pair.h>

struct wheel_velocity
{
  double left;
  double right;
};

class RobotState
{
public:
  double x{ 0 };
  double y{ 0 };
  double roll{ 0 };
  double pitch{ 0 };
  double yaw{ 0 };
  wheel_velocity velocity{};

  friend std::ostream &operator<<(std::ostream &out, const RobotState &state);

  RobotState() = default;

  explicit RobotState(const nav_msgs::Odometry::ConstPtr &msg)
  {
    setState(msg);
  }

  explicit RobotState(const geometry_msgs::PoseStamped &msg) : x{ msg.pose.position.x }, y{ msg.pose.position.y }
  {
    tf::Quaternion quaternion;
    tf::quaternionMsgToTF(msg.pose.orientation, quaternion);
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  }

  explicit RobotState(double x, double y, double yaw) : x{ x }, y{ y }, yaw{ yaw }
  {
  }

  explicit RobotState(const Eigen::Vector3d &pose) : x{ pose[0] }, y{ pose[1] }, yaw{ pose[2] }
  {
  }

  // set state using an odometry msg
  void setState(const nav_msgs::Odometry::ConstPtr &msg)
  {
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    tf::Quaternion quaternion;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quaternion);
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  }

  void setVelocity(const igvc_msgs::velocity_pairConstPtr& msg)
  {
    velocity.left = msg->left_velocity;
    velocity.right = msg->right_velocity;
  }

  void setVelocity(const igvc_msgs::velocity_pair& msg)
  {
    velocity.left = msg.left_velocity;
    velocity.right = msg.right_velocity;
  }

  // set state via a transform
  void setState(const tf::StampedTransform &transform)
  {
    x = transform.getOrigin().x();
    y = transform.getOrigin().y();
    tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
  }

  // set state via a 3D Eigen vector
  void setState(const Eigen::Vector3d &pose)
  {
    x = pose[0];
    y = pose[1];
    yaw = pose[2];
  }

  Eigen::Vector3d getVector3d() const
  {
    return { x, y, yaw };
  }

  geometry_msgs::Quaternion quat() const
  {
    return tf::createQuaternionMsgFromYaw(yaw);
  }

  bool operator==(const RobotState &other)
  {
    return std::tie(x, y, roll, pitch, yaw) == std::tie(other.x, other.y, other.roll, other.pitch, other.yaw);
  }

  /**
  returns the euclidian distance from the current robot position to a
  specified position [x2,y2]
  */
  double distTo(double x2, double y2) const
  {
    return igvc::get_distance(this->x, this->y, x2, y2);
  }

  /**
  returns the euclidian distance from the current robot position to another
  RobotState
  */
  template <class T>
  double distTo(T other) const;
};

inline std::ostream &operator<<(std::ostream &out, const RobotState &state)
{
  out << "(" << state.x << ", " << state.y << ", " << state.yaw << ")";
  return out;
}

template <>
inline double RobotState::distTo(RobotState other) const
{
  return igvc::get_distance(this->x, this->y, other.x, other.y);
}

template <>
inline double RobotState::distTo(geometry_msgs::Point other) const
{
  return igvc::get_distance(this->x, this->y, other.x, other.y);
}

#endif  // ROBOTSTATE_H
