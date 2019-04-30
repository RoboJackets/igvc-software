#define _USE_MATH_DEFINES

#include <igvc_rviz_plugins_old/waypoint_panel.h>
#include <pluginlib/class_list_macros.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <QVBoxLayout>

namespace igvc_rviz_plugins
{
WaypointPanel::WaypointPanel(QWidget *parent) : rviz::Panel(parent)
{
  ros::NodeHandle handle;

  QLabel *label = new QLabel("0");

  waypoint_subscriber = handle.subscribe<geometry_msgs::PointStamped>(
      "/waypoint", 1, boost::bind(&WaypointPanel::waypoint_callback, this, _1, label));
  robot_position_subscriber = handle.subscribe<nav_msgs::Odometry>(
      "/odometry/filtered", 1, boost::bind(&WaypointPanel::robot_position_callback, this, _1, label));

  QVBoxLayout *layout = new QVBoxLayout;
  layout->addWidget(label);
  setLayout(layout);
}

void WaypointPanel::waypoint_callback(const geometry_msgs::PointStampedConstPtr &msg, QLabel *label)
{
  way_x = msg->point.x - robot_x;
  way_y = msg->point.y - robot_y;
}

void WaypointPanel::robot_position_callback(const nav_msgs::OdometryConstPtr &msg, QLabel *label)
{
  double distance = std::hypot(way_x, way_y);
  double odomToWayPointAngle = atan2(way_y, way_x) * 180 / M_PI;

  robot_x = msg->pose.pose.position.x;
  robot_y = msg->pose.pose.position.y;

  double qx = msg->pose.pose.orientation.x;
  double qy = msg->pose.pose.orientation.y;
  double qz = msg->pose.pose.orientation.z;
  double qw = msg->pose.pose.orientation.w;

  tf::Quaternion q(qx, qy, qz, qw);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  auto text = "X: " + std::to_string(way_x) + "\n" + "Y: " + std::to_string(way_y) + "\n" +
              "Distance: " + std::to_string(distance) + "\n" + "Angle: " + std::to_string(odomToWayPointAngle - yaw) +
              " degrees\n";

  label->setText(text.c_str());
}
}  // namespace igvc_rviz_plugins_old

PLUGINLIB_EXPORT_CLASS(igvc_rviz_plugins::WaypointPanel, rviz::Panel)
