#ifndef BAT_PANEL_H
#define BAT_PANEL_H

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <QLabel>

#include <rviz/panel.h>

class QLineEdit;

namespace rviz_plugins
{
class BatPanel : public rviz::Panel
{
  Q_OBJECT
public:
  BatPanel(QWidget* parent = 0);

private:
  void batCallback(const std_msgs::UInt8& msg);

public Q_SLOTS:

protected Q_SLOTS:

protected:
  ros::Subscriber sub;
  ros::NodeHandle nh_;
  QLabel* output_topic_editor_;
};
}  // namespace rviz_plugins

#endif  // BAT_PANEL_H