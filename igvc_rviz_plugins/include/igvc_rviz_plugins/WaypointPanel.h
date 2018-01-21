#ifndef PROJECT_WAYPOINTPANEL_H
#define PROJECT_WAYPOINTPANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <QLabel>

/*
 * All of our panels need to be under the igvc_rviz_plugins namespace.
 */
namespace igvc_rviz_plugins {

/*
 * Each panel is a subclass of rviz::Panel
 */
class WaypointPanel : public rviz::Panel {
/*
 * rviz is based on QT, so our panels need to be QObjects, which requires this macro keyword.
 */
Q_OBJECT
public:

    double way_x;
    double way_y;
    double robot_x;
    double robot_y;

    /**
     * This is a standard QWidget constructor.
     * @param parent The parent widget, which will be responsible for the lifetime of this widget.
     */
    WaypointPanel(QWidget *parent = 0);

protected:
    /*
     * Be sure to make any publishers / subscribers members of the class. This will keep them alive throughout
     * the lifetime of the widget.
     */
    ros::Subscriber waypoint_subscriber;
    ros::Subscriber robot_position_subscriber;

    /**
     * Declare any ROS callbacks you need here. Be sure to create a parameter for any UI elements you need to update.
     * @param msg The ROS message that triggers this callback.
     * @param label A QT label whose text we will update based on the message contents.
     */
    void waypoint_callback(const geometry_msgs::PointStampedConstPtr &msg, QLabel *label);
    void robot_position_callback(const nav_msgs::OdometryConstPtr &msg, QLabel *label);

    /**
     * If you need to paint custom graphics on your panel, uncomment and implement the paintEvent method.
     * You can find out more about this method here: http://doc.qt.io/qt-5/qwidget.html#paintEvent
     */
//    void paintEvent(QPaintEvent *event) override;

};

}

#endif //PROJECT_WAYPOINTPANEL_H
