#ifndef ESTOP_PANEL_H
#define ESTOP_PANEL_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <QLabel>

#include <rviz/panel.h>

class QLineEdit;

namespace rviz_plugins
{
class EStopPanel : public rviz::Panel
{
  Q_OBJECT
public:
  EStopPanel(QWidget* parent = 0);

private:
  void subCallback(const std_msgs::Bool& msg);

protected:
  ros::Subscriber sub;
  ros::NodeHandle nh_;
  QLabel* output_topic_editor_;
};
}  // namespace rviz_plugins

#endif  // ESTOP_PANEL_H