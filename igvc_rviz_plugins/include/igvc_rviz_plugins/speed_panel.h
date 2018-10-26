#ifndef SPEED_PANEL_H
#define SPEED_PANEL_H

#include <ros/ros.h>
#include <QApplication>
#include <QLabel>

#include <igvc_msgs/velocity_pair.h>

#include <rviz/panel.h>
#include "speedometer.h"
class QLineEdit;

namespace rviz_plugins
{
class SpeedPanel : public rviz::Panel
{
  Q_OBJECT
public:
  SpeedPanel(QWidget* parent = 0);

private:
  void subCallback(const igvc_msgs::velocity_pair& msg);
  double speed;
  Speedometer* speedometer;

public Q_SLOTS:

protected Q_SLOTS:

protected:
  ros::Subscriber sub;
  ros::NodeHandle nh_;
  QLabel* velocity_label;
  QLabel* velocity_right_label;
  QLabel* velocity_left_label;
};
}  // namespace rviz_plugins

#endif  // SPEED_PANEL_H