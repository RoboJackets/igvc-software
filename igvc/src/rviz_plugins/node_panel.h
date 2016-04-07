#ifndef BARE_PANEL_H
#define BARE_PANEL_H

#include <ros/ros.h>
#include <QLabel>
#include <std_msgs/UInt8.h>

#include <rviz/panel.h>

#include <ctime>

#include "my_thread.h"

#include <QPushButton>
#include <QComboBox>

class NodePanel: public rviz::Panel
{
  Q_OBJECT
  public:
    NodePanel( QWidget* parent = 0 );

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

#endif // BARE_PANEL_H