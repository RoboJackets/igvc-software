#ifndef NODE_PANEL_H
#define NODE_PANEL_H

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <QLabel>

#include <rviz/panel.h>

#include <ctime>

#include "my_thread.h"

#include "num_button.h"

#include <QComboBox>
#include <QPushButton>
#include <QVBoxLayout>

namespace rviz_plugins
{
class NodePanel : public rviz::Panel
{
  Q_OBJECT
public:
  NodePanel(QWidget* parent = 0);

private Q_SLOTS:
  void closeNode(int num);
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
  NumButton* close_btn;
  NumButton* test_btn;
  std::vector<MyThread*> myThreads;
  int curThread;
  QVBoxLayout* layout;
};
}  // namespace rviz_plugins

#endif  // NODE_PANEL_H