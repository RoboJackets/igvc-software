#ifndef PROJECT_WAYPOINTPANEL_H
#define PROJECT_WAYPOINTPANEL_H

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <QLabel>

namespace igvc_rviz_plugins
{
class WaypointPanel : public rviz::Panel
{
  Q_OBJECT
public:
  double way_x;
  double way_y;
  double robot_x;
  double robot_y;

  WaypointPanel(QWidget *parent = 0);

protected:
  ros::Subscriber waypoint_subscriber;
  ros::Subscriber robot_position_subscriber;

  void waypoint_callback(const geometry_msgs::PointStampedConstPtr &msg, QLabel *label);
  void robot_position_callback(const nav_msgs::OdometryConstPtr &msg, QLabel *label);
};
}  // namespace igvc_rviz_plugins

#endif
