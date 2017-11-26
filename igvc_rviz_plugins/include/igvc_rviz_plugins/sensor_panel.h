#ifndef SENSORPANEL_H
#define SENSORPANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <QLabel>
#include <QBasicTimer>
#include <QObject>

/*
 * All of our panels need to be under the igvc_rviz_plugins namespace.
 */
namespace igvc_rviz_plugins {

/*
 * Each panel is a subclass of rviz::Panel
 */
class sensor_panel : public rviz::Panel {
/*
 * rviz is based on QT, so our panels need to be QObjects, which requires this macro keyword.
 */
Q_OBJECT
public:

    /**
     * This is a standard QWidget constructor.
     * @param parent The parent widget, which will be responsible for the lifetime of this widget.
     */
    sensor_panel(QWidget *parent = 0);

protected:
    /*
     * Be sure to make any publishers / subscribers members of the class. This will keep them alive throughout
     * the lifetime of the widget.
     */
    ros::Subscriber imu_sub;
    ros::Subscriber lidar_sub;
    ros::Subscriber gps_sub;
    ros::Subscriber camCenter_sub;
	ros::Subscriber camLeft_sub;
	ros::Subscriber camRight_sub;

	QBasicTimer *sensor_timer;
	QLabel *label, *label2, *label3, *label4, *label5, *label6;
    


    /**
     * Declare any ROS callbacks you need here. Be sure to create a parameter for any UI elements you need to update.
     * @param msg The ROS message that triggers this callback.
     * @param label A QT label whose text we will update based on the message contents.
     */
    void imu_callback(const sensor_msgs::ImuConstPtr &msg, QLabel *label);
    void lidar_callback(const sensor_msgs::PointCloud2ConstPtr &msg, QLabel *label);
    void gps_callback(const sensor_msgs::ImuConstPtr &msg, QLabel *label);
    void camCenter_callback(const sensor_msgs::ImageConstPtr &msg, QLabel *label);
	void camLeft_callback(const sensor_msgs::ImageConstPtr &msg, QLabel *label);
	void camRight_callback(const sensor_msgs::ImageConstPtr &msg, QLabel *label);
	void gps2_callback(const sensor_msgs::NavSatFixConstPtr &msg, QLabel *label);
    /**
     * If you need to paint custom graphics on your panel, uncomment and implement the paintEvent method.
     * You can find out more about this method here: http://doc.qt.io/qt-5/qwidget.html#paintEvent
     */
//    void paintEvent(QPaintEvent *event) override;

private:
	void timer();
};

}

#endif //SENSORPANEL_H
