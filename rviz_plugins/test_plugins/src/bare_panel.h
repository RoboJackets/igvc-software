#ifndef BARE_PANEL_H
#define BARE_PANEL_H

#include <ros/ros.h>

#include <rviz/panel.h>

class QLineEdit;

namespace test_plugins
{

  class BarePanel: public rviz::Panel
  {
    Q_OBJECT
    public:
      BarePanel( QWidget* parent = 0 );

    public Q_SLOTS:

    protected Q_SLOTS:

    protected:
      ros::Subscriber sub;
      ros::NodeHandle nh_;
      QLineEdit* output_topic_editor_;

  };

}

#endif // BARE_PANEL_H