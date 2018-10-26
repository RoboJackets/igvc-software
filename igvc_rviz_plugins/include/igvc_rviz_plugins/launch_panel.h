#ifndef LAUNCH_PANEL_H
#define LAUNCH_PANEL_H

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <QLabel>

#include <rviz/panel.h>

#include <ctime>

#include "my_thread.h"

#include <QComboBox>
#include <QPushButton>

namespace rviz_plugins
{
class LaunchPanel : public rviz::Panel
{
  Q_OBJECT
public:
  LaunchPanel(QWidget* parent = 0);

private Q_SLOTS:
  void handleButton();

public Q_SLOTS:

protected Q_SLOTS:

protected:
  ros::Subscriber sub;
  ros::NodeHandle nh_;
  QComboBox* output_topic_editor_;
  time_t start;
  bool doOnce;
  QPushButton* m_button;
};
}  // namespace rviz_plugins

#endif  // LAUNCH_PANEL_H