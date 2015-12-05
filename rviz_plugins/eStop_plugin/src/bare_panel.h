#ifndef BARE_PANEL_H
#define BARE_PANEL_H

#include <ros/ros.h>
#include <QLabel>
#include <std_msgs/Bool.h>

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
      void subCallback(const std_msgs::Bool &msg);

    public Q_SLOTS:

    protected Q_SLOTS:

    protected:
      ros::Subscriber sub;
      ros::NodeHandle nh_;
      QLabel* output_topic_editor_;

  };

}

#endif // BARE_PANEL_H