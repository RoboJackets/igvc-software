#include <igvc_utils/robot_state.h>

RobotState::RobotState(const nav_msgs::Odometry::ConstPtr &msg)
{
  setState(msg);
}

RobotState::RobotState(const geometry_msgs::PoseStamped &msg) : x{ msg.pose.position.x }, y{ msg.pose.position.y }
{
  tf::Quaternion quaternion;
  tf::quaternionMsgToTF(msg.pose.orientation, quaternion);
  tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
}

RobotState::RobotState(double x, double y, double yaw) : x{ x }, y{ y }, yaw{ yaw }
{
}

RobotState::RobotState(const Eigen::Vector3d &pose) : x{ pose[0] }, y{ pose[1] }, yaw{ pose[2] }
{
}

void RobotState::setState(const nav_msgs::Odometry::ConstPtr &msg)
{
  x = msg->pose.pose.position.x;
  y = msg->pose.pose.position.y;
  tf::Quaternion quaternion;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, quaternion);
  tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
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
void RobotState::setState(const tf::StampedTransform &transform)
{
  x = transform.getOrigin().x();
  y = transform.getOrigin().y();
  tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
}

// set state via a 3D Eigen vector
void RobotState::setState(const Eigen::Vector3d &pose)
{
  x = pose[0];
  y = pose[1];
  yaw = pose[2];
}

Eigen::Vector3d RobotState::getVector3d() const
{
  return { x, y, yaw };
}

geometry_msgs::Quaternion RobotState::quat() const
{
  return tf::createQuaternionMsgFromYaw(yaw);
}

geometry_msgs::PoseStamped RobotState::toPose(ros::Time stamp) const
{
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.pose.position.x = x;
  pose_stamped.pose.position.y = y;
  pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
  return pose_stamped;
}

igvc_msgs::trajectory_point RobotState::toTrajectoryPoint(ros::Time stamp, const RobotControl &control,
                                                          double axle_length) const
{
  igvc_msgs::trajectory_point point;
  point.header.stamp = stamp;
  point.pose.position.x = x;
  point.pose.position.y = y;
  point.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
  const auto [k, v] = control.toKV(axle_length);
  point.curvature = k;
  point.velocity = v;
  return point;
}

void RobotState::propogateState(const RobotControl &robot_control, double dt)
{
  auto [k, v] = robot_control.toKV();
  double w = k * v;

  if (std::abs(w) > 1e-10)
  {
    // calculate instantaneous center of curvature (ICC = [ICCx, ICCy])
    double R = v / w;
    double ICCx = x - (R * sin(yaw));
    double ICCy = y + (R * cos(yaw));

    using namespace Eigen;
    Matrix3d T;
    double wdt = w * dt;
    T << cos(wdt), -sin(wdt), 0, sin(wdt), cos(wdt), 0, 0, 0, 1;
    Vector3d a(x - ICCx, y - ICCy, yaw);
    Vector3d b = T * a;
    Vector3d c = b + Vector3d(ICCx, ICCy, wdt);

    x = c[0];
    y = c[1];
    yaw = c[2];
    igvc::fit_to_polar(yaw);
  }
  else
  {
    x = x + cos(yaw) * v * dt;
    y = y + cos(yaw) * v * dt;
  }
  velocity.left = robot_control.left_;
  velocity.right = robot_control.right_;
}

bool RobotState::operator==(const RobotState &other)
{
  return std::tie(x, y, roll, pitch, yaw) == std::tie(other.x, other.y, other.roll, other.pitch, other.yaw);
}

