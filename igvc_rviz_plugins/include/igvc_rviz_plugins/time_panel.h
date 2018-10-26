#ifndef TIME_PANEL_H
#define TIME_PANEL_H

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <QLabel>

#include <rviz/panel.h>

#include <ctime>

class QLineEdit;

namespace rviz_plugins
{
class TimePanel : public rviz::Panel
{
  Q_OBJECT
public:
  TimePanel(QWidget* parent = 0);

private:
  void timeCallback(const std_msgs::UInt8& msg);

public Q_SLOTS:

protected Q_SLOTS:

protected:
  ros::Subscriber sub;
  ros::NodeHandle nh_;
  QLabel* output_topic_editor_;
  time_t start;
};
}  // namespace rviz_plugins

#endif  // TIME_PANEL_H