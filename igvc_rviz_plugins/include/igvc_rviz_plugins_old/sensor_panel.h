#ifndef SENSORPANEL_H
#define SENSORPANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <QLabel>
#include <QObject>
#include <QString>
#include <QTimer>
#include <string>

#define INTERVAL 250  // in milliseconds

/*
 * All of our panels need to be under the igvc_rviz_plugins_old namespace.
 */
namespace igvc_rviz_plugins
{
/*
 * Each panel is a subclass of rviz::Panel
 */
class sensor_panel : public rviz::Panel
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
  sensor_panel(QWidget *parent = 0);

public slots:
  void reset_labels();

protected:
  /*
   * Subscribers for each sensor.
   */
  ros::Subscriber imu_sub;
  ros::Subscriber lidar_sub;
  ros::Subscriber gps_sub;
  ros::Subscriber cam_center_sub;

  QTimer *sensor_timer;
  std::map<std::string, QLabel *> labels;

  const QString green = "QLabel { background-color : green; color : black; font: bold 14px;}";
  const QString red = "QLabel { background-color : red; color : black; font: bold 14px; }";
  /**
   * Callbacks for each sensor.
   */
  void imu_callback(const sensor_msgs::ImuConstPtr &msg, QLabel *ls);
  void lidar_callback(const sensor_msgs::PointCloud2ConstPtr &msg, QLabel *ls);
  void cam_center_callback(const sensor_msgs::ImageConstPtr &msg, QLabel *ls);
  void gps_callback(const sensor_msgs::NavSatFixConstPtr &msg, QLabel *ls);
};
}  // namespace igvc_rviz_plugins

#endif  // SENSORPANEL_H
