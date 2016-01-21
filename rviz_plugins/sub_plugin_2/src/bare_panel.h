#ifndef BARE_PANEL_H
#define BARE_PANEL_H

#include <ros/ros.h>
#include <QLabel>
#include <QApplication>

#include <igvc_msgs/velocity_pair.h>

#include <rviz/panel.h>

class QLineEdit;

namespace bare
{

  class BarePanel: public rviz::Panel
  {
    Q_OBJECT
    public:
      BarePanel( QWidget* parent = 0 );

    private:
      void subCallback(const igvc_msgs::velocity_pair &msg);

    public Q_SLOTS:

    protected Q_SLOTS:

    protected:
      ros::Subscriber sub;
      ros::NodeHandle nh_;
      QLabel* velocity_label;
      QLabel* velocity_right_label;
      QLabel* velocity_left_label;

  };

}

#endif // BARE_PANEL_H
