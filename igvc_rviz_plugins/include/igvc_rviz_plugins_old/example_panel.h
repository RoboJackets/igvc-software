#ifndef PROJECT_EXAMPLEPANEL_H
#define PROJECT_EXAMPLEPANEL_H

#include <igvc_msgs/velocity_quad.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <QLabel>

/*
 * All of our panels need to be under the igvc_rviz_plugins_old namespace.
 */
namespace igvc_rviz_plugins
{
/*
 * Each panel is a subclass of rviz::Panel
 */
class ExamplePanel : public rviz::Panel
{
  /*
   * rviz is based on QT, so our panels need to be QObjects, which requires this macro keyword.
   */
  Q_OBJECT
public:
  /**
   * This is a standard QWidget constructor.
   * @param parent The parent widget, which will be responsible for the lifetime of this widget.
   */
  ExamplePanel(QWidget *parent = 0);

protected:
  /*
   * Be sure to make any publishers / subscribers members of the class. This will keep them alive throughout
   * the lifetime of the widget.
   */
  ros::Subscriber speed_subscriber;

  /**
   * Declare any ROS callbacks you need here. Be sure to create a parameter for any UI elements you need to update.
   * @param msg The ROS message that triggers this callback.
   * @param label A QT label whose text we will update based on the message contents.
   */
  void speed_callback(const igvc_msgs::velocity_quadConstPtr &msg, QLabel *label);

  /**
   * If you need to paint custom graphics on your panel, uncomment and implement the paintEvent method.
   * You can find out more about this method here: http://doc.qt.io/qt-5/qwidget.html#paintEvent
   */
  //    void paintEvent(QPaintEvent *event) override;
};
}  // namespace igvc_rviz_plugins

#endif  // PROJECT_EXAMPLEPANEL_H
