#ifndef LAUNCH_PANEL_H
#define LAUNCH_PANEL_H

#include <ros/ros.h>
#include <QLabel>
#include <std_msgs/UInt8.h>

#include <rviz/panel.h>

#include <ctime>

#include "my_thread.h"

#include <QPushButton>
#include <QComboBox>

namespace rviz_plugins
{

class LaunchPanel: public rviz::Panel
{
  Q_OBJECT
  public:
    LaunchPanel( QWidget* parent = 0 );

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
    QPushButton *m_button;

};

}

#endif // LAUNCH_PANEL_H