#include <igvc_utils/robot_state.h>

RobotState::RobotState(const nav_msgs::Odometry::ConstPtr &msg)
{
  setState(msg);
}

RobotState::RobotState(const geometry_msgs::PoseStamped &msg)
{
  tf::poseMsgToTF(msg.pose, transform);
}

RobotState::RobotState(double x, double y, double yaw)
{
  set_x(x);
  set_y(y);
  set_yaw(yaw);
}

RobotState::RobotState(const Eigen::Vector3d &pose)
{
  setState(pose);
}

double RobotState::x() const
{
  return transform.getOrigin().x();
}

double RobotState::y() const
{
  return transform.getOrigin().y();
}

double RobotState::z() const
{
  return transform.getOrigin().z();
}

double RobotState::roll() const
{
  double r, p, y;
  transform.getBasis().getRPY(r, p, y);
  return r;
}

double RobotState::pitch() const
{
  double r, p, y;
  transform.getBasis().getRPY(r, p, y);
  return p;
}

double RobotState::yaw() const
{
  double r, p, y;
  transform.getBasis().getRPY(r, p, y);
  return y;
}

double RobotState::quat_x() const
{
  return transform.getRotation().x();
}
double RobotState::quat_y() const
{
  return transform.getRotation().y();
}
double RobotState::quat_z() const
{
  return transform.getRotation().z();
}
double RobotState::quat_w() const
{
  return transform.getRotation().w();
}

void RobotState::set_x(double x)
{
  transform.setOrigin(tf::Vector3(x, y(), z()));
}
void RobotState::set_y(double y)
{
  transform.setOrigin(tf::Vector3(x(), y, z()));
}
void RobotState::set_z(double z)
{
  transform.setOrigin(tf::Vector3(x(), y(), z));
}
void RobotState::set_roll(double roll)
{
  tf::Matrix3x3 t = transform.getBasis();
  t.setRPY(roll, pitch(), yaw());
  transform.setBasis(t);
}
void RobotState::set_pitch(double pitch)
{
  tf::Matrix3x3 t = transform.getBasis();
  t.setRPY(roll(), pitch, yaw());
  transform.setBasis(t);
}

void RobotState::set_yaw(double yaw)
{
  tf::Matrix3x3 t = transform.getBasis();
  t.setRPY(roll(), pitch(), yaw);
  transform.setBasis(t);
}

void RobotState::setState(const nav_msgs::Odometry::ConstPtr &msg)
{
  tf::poseMsgToTF(msg->pose.pose, transform);
}

void RobotState::setVelocity(const igvc_msgs::velocity_pairConstPtr &msg)
{
  velocity.left = msg->left_velocity;
  velocity.right = msg->right_velocity;
}

void RobotState::setVelocity(const igvc_msgs::velocity_pair &msg)
{
  velocity.left = msg.left_velocity;
  velocity.right = msg.right_velocity;
}

// set state via a transform
void RobotState::setState(const tf::StampedTransform &stamped_transform)
{
  transform = tf::Transform(stamped_transform);
}

// set state via a 3D Eigen vector
void RobotState::setState(const Eigen::Vector3d &pose)
{
  set_x(pose[0]);
  set_y(pose[1]);
  set_yaw(pose[2]);
}

Eigen::Vector3d RobotState::getVector3d() const
{
  return { x(), y(), yaw() };
}

geometry_msgs::Quaternion RobotState::quat() const
{
  geometry_msgs::Quaternion quat;
  tf::quaternionTFToMsg(transform.getRotation(), quat);
  return quat;
}

geometry_msgs::Pose RobotState::toPose() const
{
  geometry_msgs::Pose pose;
  tf::poseTFToMsg(transform, pose);
  return pose;
}

igvc_msgs::trajectory_point RobotState::toTrajectoryPoint(ros::Time stamp, const RobotControl &control,
                                                          double axle_length) const
{
  igvc_msgs::trajectory_point point;
  point.header.stamp = stamp;
  point.pose = toPose();

  const auto [k, v] = control.toKV(axle_length);
  point.curvature = k;
  point.velocity = v;
  return point;
}

void RobotState::propagateState(const RobotControl &robot_control, double axle_length, double dt)
{
  auto [k, v] = robot_control.toKV(axle_length);
  double w = k * v;

  if (std::abs(w) > 1e-10)
  {
    auto [ICC_x, ICC_y] = getICC(robot_control, axle_length);

    using namespace Eigen;
    Matrix3d T;
    double wdt = w * dt;
    T << cos(wdt), -sin(wdt), 0, sin(wdt), cos(wdt), 0, 0, 0, 1;
    Vector3d a(x() - ICC_x, y() - ICC_y, yaw());
    Vector3d b = T * a;
    Vector3d c = b + Vector3d(ICC_x, ICC_y, wdt);

    set_x(c[0]);
    set_y(c[1]);
    set_yaw(c[2]);
  }
  else
  {
    set_x(x() + cos(yaw()) * v * dt);
    set_y(y() + cos(yaw()) * v * dt);
  }
  velocity.left = robot_control.left_;
  velocity.right = robot_control.right_;
}

std::pair<double, double> RobotState::getICC(const RobotControl &robot_control, double axle_length) const
{
  auto [k, v] = robot_control.toKV(axle_length);
  double w = k * v;
  double R = v / w;
  double ICC_x = x() - (R * sin(yaw()));
  double ICC_y = y() + (R * cos(yaw()));

  return std::make_pair(ICC_x, ICC_y);
}

bool RobotState::operator==(const RobotState &other)
{
  return transform == other.transform;
}

double RobotState::getArcLength(const igvc_msgs::trajectory_point &a, const igvc_msgs::trajectory_point &b)
{
  if (std::abs(a.curvature) > 1e-8)
  {
    double R = 1 / a.curvature;
    double current_yaw = tf::getYaw(a.pose.orientation);
    double next_yaw = tf::getYaw(b.pose.orientation);

    double d_theta = next_yaw - current_yaw;
    return std::abs(d_theta * R);
  }
  else
  {
    double dx = a.pose.position.x - b.pose.position.x;
    double dy = a.pose.position.y - b.pose.position.y;
    return std::hypot(dx, dy);
  }
}
