#ifndef ROBOTSTATE_H
#define ROBOTSTATE_H

#include <igvc_msgs/trajectory_point.h>
#include <igvc_msgs/velocity_pair.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <igvc_utils/NodeUtils.hpp>
#include "robot_control.h"

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

  explicit RobotState(const nav_msgs::Odometry::ConstPtr &msg);
  explicit RobotState(const geometry_msgs::PoseStamped &msg);
  explicit RobotState(double x, double y, double yaw);
  explicit RobotState(const Eigen::Vector3d &pose);

  void setVelocity(const igvc_msgs::velocity_pairConstPtr &msg);
  void setVelocity(const igvc_msgs::velocity_pair &msg);

  void setState(const tf::StampedTransform &transform);
  void setState(const Eigen::Vector3d &pose);
  void setState(const nav_msgs::Odometry::ConstPtr &msg);

  Eigen::Vector3d getVector3d() const;

  geometry_msgs::Quaternion quat() const;

  geometry_msgs::PoseStamped toPose(ros::Time stamp = ros::Time::now()) const;

  igvc_msgs::trajectory_point toTrajectoryPoint(ros::Time stamp, const RobotControl &control, double axle_length) const;

  /**
   * Propogates the current state according to controls robot_control for time dt
   * @param robot_control controls to be applied.
   * @param dt duration the controls are applied.
   */
  void propogateState(const RobotControl &robot_control, double dt);

  bool operator==(const RobotState &other);

  /**
   * returns the euclidean distance from the current position to the passed in position
   * @param x2
   * @param y2
   * @return
   */
  double distTo(double x2, double y2) const;

  double distTo(RobotState other) const;

  double distTo(geometry_msgs::Point other) const;

  double linearVelocity() const;
};

inline std::ostream &operator<<(std::ostream &out, const RobotState &state)
{
  out << "(" << state.x << ", " << state.y << ", " << state.yaw << ")\tv = (" << state.velocity.left << ", "
      << state.velocity.right << ")";
  return out;
}

inline double RobotState::distTo(double x2, double y2) const
{
  return igvc::get_distance(this->x, this->y, x2, y2);
}

inline double RobotState::distTo(RobotState other) const
{
  return igvc::get_distance(this->x, this->y, other.x, other.y);
}

inline double RobotState::distTo(geometry_msgs::Point other) const
{
  return igvc::get_distance(this->x, this->y, other.x, other.y);
}

inline double RobotState::linearVelocity() const {
  return (velocity.left + velocity.right) / 2;
}

#endif  // ROBOTSTATE_H
