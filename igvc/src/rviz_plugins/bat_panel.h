#ifndef BARE_PANEL_H
#define BARE_PANEL_H

#include <ros/ros.h>
#include <QLabel>
#include <std_msgs/UInt8.h>

#include <rviz/panel.h>

class QLineEdit;

class BatPanel: public rviz::Panel
{
  Q_OBJECT
  public:
    BatPanel( QWidget* parent = 0 );

  private:
    void batCallback(const std_msgs::UInt8 &msg);

  public Q_SLOTS:

  protected Q_SLOTS:

  protected:
    ros::Subscriber sub;
    ros::NodeHandle nh_;
    QLabel* output_topic_editor_;

};

#endif // BARE_PANEL_H