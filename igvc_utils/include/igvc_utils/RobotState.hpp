#ifndef ROBOTSTATE_H
#define ROBOTSTATE_H

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <igvc_utils/NodeUtils.hpp>

class RobotState
{
public:
  // double x{ 0 };
  // double y{ 0 };
  // double z{ 0 };
  // double roll{ 0 };
  // double pitch{ 0 };
  // double yaw{ 0 };
  tf::Transform transform;
  ros::Time stamp{ 0 };

  friend std::ostream &operator<<(std::ostream &out, const RobotState &state);

  RobotState() = default;

  explicit RobotState(const nav_msgs::Odometry::ConstPtr &msg)
  {
    setState(msg);
  }

  explicit RobotState(double x, double y, double yaw)
  {
    transform.setOrigin(tf::Vector3(x, y, 0));
    tf::Matrix3x3 rot;
    rot.setRPY(0, 0, yaw);
    transform.setBasis(rot);
  }

  explicit RobotState(const tf::StampedTransform &stamped_transform, ros::Time stamp)
      : transform(stamped_transform), stamp(stamp)
  {
  }

  explicit RobotState(const tf::Transform& transform) : transform(transform) { }

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

  // set state using an odometry msg
  void setState(const nav_msgs::Odometry::ConstPtr &msg)
  {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;
    //    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    tf::Quaternion quaternion;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quaternion);
    transform.setRotation(quaternion);
    transform.setOrigin(tf::Vector3(x, y, z));
  }

  // set state via a transform
  void setState(const tf::StampedTransform &transform)
  {
    this->transform = transform;
  }

  // set state via a 3D Eigen vector
  void setState(const Eigen::Vector3d pose)
  {
    transform.setOrigin(tf::Vector3(pose[0], pose[1], 0));
    tf::Matrix3x3 rot = transform.getBasis();
    double r, p, y;
    rot.getRPY(r, p, y);
    rot.setEulerYPR(pose[2], p, r);
    transform.setBasis(rot);
  }

  Eigen::Vector3d getVector3d() const
  {
    return { x(), y(), yaw() };
  }

  geometry_msgs::Quaternion quat() const
  {
    geometry_msgs::Quaternion quat;
    tf::quaternionTFToMsg(transform.getRotation(), quat);
    return quat;
  }

  bool operator==(const RobotState &other)
  {
    return transform == other.transform;
  }

  void operator*=(const tf::Transform other)
  {
    transform *= other;
  }

  RobotState operator*(const RobotState& other) const
  {
    RobotState combined(transform * other.transform);
    return combined;
  }

  /**
   * Adds the state vector to the current transform. Use case is adding noise.
   * @param state
   * @return
   */
  void operator+=(Eigen::Matrix<double, 6, 1> state)
  {
    transform.setOrigin(tf::Vector3(x() + state(0), y() + state(1), z() + state(2)));
    double r, p, y;
    transform.getBasis().getRPY(r, p, y);
    transform.getBasis().setRPY(r + state(3), p + state(4), y + state(5));
  }

  /**
  returns the euclidian distance from the current robot position to a
  specified position [x2,y2]
  */
  double distTo(double x2, double y2) const
  {
    return igvc::get_distance(this->x(), this->y(), x2, y2);
  }

  /**
  returns the euclidian distance from the current robot position to another
  RobotState
  */
  template <class T>
  double distTo(T other) const;
};

template <>
inline double RobotState::distTo(RobotState other) const {
  return igvc::get_distance(this->x(), this->y(), other.x(), other.y());
}

template <>
inline double RobotState::distTo(geometry_msgs::Point other) const {
  return igvc::get_distance(this->x(), this->y(), other.x, other.y);
}

inline double RobotState::x() const
{
  return transform.getOrigin().x();
}

inline double RobotState::y() const
{
  return transform.getOrigin().y();
}

inline double RobotState::z() const
{
  return transform.getOrigin().z();
}

inline double RobotState::roll() const
{
  double r, p, y;
  transform.getBasis().getRPY(r, p, y);
  return r;
}

inline double RobotState::pitch() const
{
  double r, p, y;
  transform.getBasis().getRPY(r, p, y);
  return p;
}

inline double RobotState::yaw() const
{
  double r, p, y;
  transform.getBasis().getRPY(r, p, y);
  return y;
}

inline double RobotState::quat_x() const { return transform.getRotation().x(); }
inline double RobotState::quat_y() const { return transform.getRotation().y(); }
inline double RobotState::quat_z() const { return transform.getRotation().z(); }
inline double RobotState::quat_w() const { return transform.getRotation().w(); }


inline std::ostream &operator<<(std::ostream &out, const RobotState &state)
{
  out << "(" << state.x() << ", " << state.y() << ", " << state.yaw() << ")";
  return out;
}

inline void RobotState::set_x(double x) { transform.setOrigin(tf::Vector3(x, y(), z())); }
inline void RobotState::set_y(double y) { transform.setOrigin(tf::Vector3(x(), y, z())); }
inline void RobotState::set_z(double z) { transform.setOrigin(tf::Vector3(x(), y(), z)); }
inline void RobotState::set_roll(double roll) {
  tf::Matrix3x3 t = transform.getBasis();
  t.setRPY(roll, pitch(), yaw());
  transform.setBasis(t);
}
inline void RobotState::set_pitch(double pitch) {
  tf::Matrix3x3 t = transform.getBasis();
  t.setRPY(roll(), pitch, yaw());
  transform.setBasis(t);
}

inline void RobotState::set_yaw(double yaw) {
  tf::Matrix3x3 t = transform.getBasis();
  t.setRPY(roll(), pitch(), yaw);
  transform.setBasis(t);
}

#endif  // ROBOTSTATE_H