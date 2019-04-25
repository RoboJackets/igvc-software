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
  tf::Transform transform{};
  wheel_velocity velocity{};

  friend std::ostream &operator<<(std::ostream &out, const RobotState &state);

  RobotState() = default;

  explicit RobotState(const nav_msgs::Odometry::ConstPtr &msg);
  explicit RobotState(const geometry_msgs::PoseStamped &msg);
  explicit RobotState(double x, double y, double yaw);
  explicit RobotState(const Eigen::Vector3d &pose);

  double x() const;
  double y() const;
  double z() const;
  double roll() const;
  double pitch() const;
  double yaw() const;
  double quat_x() const;
  double quat_y() const;
  double quat_z() const;
  double quat_w() const;

  void set_x(double x);
  void set_y(double y);
  void set_z(double z);
  void set_roll(double roll);
  void set_pitch(double pitch);
  void set_yaw(double yaw);

  void setVelocity(const igvc_msgs::velocity_pairConstPtr &msg);
  void setVelocity(const igvc_msgs::velocity_pair &msg);

  void setState(const tf::StampedTransform &transform);
  void setState(const Eigen::Vector3d &pose);
  void setState(const nav_msgs::Odometry::ConstPtr &msg);

  Eigen::Vector3d getVector3d() const;

  geometry_msgs::Quaternion quat() const;

  geometry_msgs::Pose toPose() const;

  igvc_msgs::trajectory_point toTrajectoryPoint(ros::Time stamp, const RobotControl &control, double axle_length) const;

  /**
   * Propogates the current state according to controls robot_control for time dt
   * @param robot_control controls to be applied.
   * @param dt duration the controls are applied.
   */
  void propagateState(const RobotControl &robot_control, double dt);

  std::pair<double, double> getICC(const RobotControl &robot_control) const;

  bool operator==(const RobotState &other);

  /**
   * returns the euclidean distance from the current position to the passed in position
   * @param x2
   * @param y2
   * @return
   */
  double distTo(double x2, double y2) const;

  double distTo(const RobotState &other) const;

  double distTo(const geometry_msgs::Point &other) const;

  double linearVelocity() const;

  /**
   * Returns the arc length between trajectory_points a and b, given that b was the result of
   * applying a.velocity with curvature a.curvature. If curavture is small, this is approximated
   * as a straight line. Otherwise, this distance is calculated as an arc with radius 1/a.curvature
   * @param a
   * @param b
   * @return
   */
  static double getArcLength(const igvc_msgs::trajectory_point &a, const igvc_msgs::trajectory_point &b);
};

inline std::ostream &operator<<(std::ostream &out, const RobotState &state)
{
  out << "(" << state.x() << ", " << state.y() << ", " << state.yaw() << ")\tv = (" << state.velocity.left << ", "
      << state.velocity.right << ")";
  return out;
}

inline double RobotState::distTo(double x2, double y2) const
{
  return std::hypot(this->x() - x2, this->y() - y2);
}

inline double RobotState::distTo(const RobotState &other) const
{
  return distTo(other.x(), other.y());
}

inline double RobotState::distTo(const geometry_msgs::Point &other) const
{
  return distTo(other.x, other.y);
}

inline double RobotState::linearVelocity() const
{
  return (velocity.left + velocity.right) / 2;
}

#endif  // ROBOTSTATE_H
