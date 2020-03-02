#include <igvc_rviz_plugins_old/sensor_panel.h>
#include <pluginlib/class_list_macros.h>
#include <QVBoxLayout>
/*
 * Just as in the header, everything needs to happen in the igvc_rviz_plugins_old namespace.
 */
namespace igvc_rviz_plugins
{
sensor_panel::sensor_panel(QWidget *parent) : rviz::Panel(parent)  // Base class constructor
{
  // Init all variables
  ros::NodeHandle nh;
  QVBoxLayout *layout = new QVBoxLayout();  // layouts hold widgets

  // all labels for displaying data
  labels["IMU"] = new QLabel("IMU: Disabled\n");
  labels["Lidar"] = new QLabel("Lidar: Disabled\n");
  labels["Center Camera"] = new QLabel("Center Camera: Disabled\n");
  labels["GPS"] = new QLabel("GPS: Disabled\n");

  /*
   *Makes timer that repeats every interval seconds
   *When timer goes off, it calls the timer method
   */
  sensor_timer = new QTimer();
  sensor_timer->setSingleShot(false);   // will repeat
  sensor_timer->setInterval(INTERVAL);  // goes off every 250 milliseconds

  // calls reset_labels whenever sensor_timer goes off
  QObject::connect(sensor_timer, SIGNAL(timeout()), this, SLOT(reset_labels()));

  /* Initialize subscribers to listen to topic corresponding with each sensor.
   * use boost::bind to update labels with each call.
   */
  imu_sub =
      nh.subscribe<sensor_msgs::Imu>("/imu", 1, boost::bind(&sensor_panel::imu_callback, this, _1, labels["IMU"]));
  lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>(
      "/scan/pointcloud", 1, boost::bind(&sensor_panel::lidar_callback, this, _1, labels["Lidar"]));
  cam_center_sub = nh.subscribe<sensor_msgs::Image>(
      "/usb_cam_center/image_raw", 1,
      boost::bind(&sensor_panel::cam_center_callback, this, _1, labels["Center Camera"]));
  gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("/fix", 1,
                                                 boost::bind(&sensor_panel::gps_callback, this, _1, labels["GPS"]));

  // adds all the widgets to the layout
  for (std::pair<std::string, QLabel *> pear : labels)
  {
    pear.second->setStyleSheet(red);
    layout->addWidget(pear.second);
  }
  setLayout(layout);
  sensor_timer->start();
}

void sensor_panel::imu_callback(const sensor_msgs::ImuConstPtr &msg, QLabel *label)
{
  label->setText("IMU: Enabled ");
  label->setStyleSheet(green);
}

void sensor_panel::lidar_callback(const sensor_msgs::PointCloud2ConstPtr &msg, QLabel *label)
{
  label->setText("Lidar: Enabled ");
  label->setStyleSheet(green);
}

void sensor_panel::cam_center_callback(const sensor_msgs::ImageConstPtr &msg, QLabel *label)
{
  label->setText("Center Camera: Enabled ");
  label->setStyleSheet(green);
}

void sensor_panel::gps_callback(const sensor_msgs::NavSatFixConstPtr &msg, QLabel *label)
{
  label->setText("GPS: Enabled ");
  label->setStyleSheet(green);
}

/*
 *Uses trailing space to figure out if a sensor is inactive
 *No trailing space means the sensor was inactive for the whole timer cycle
 */
void sensor_panel::reset_labels()
{
  int index;
  for (std::pair<std::string, QLabel *> pear : labels)
  {
    // convert QString to std::string
    std::string str = pear.second->text().toUtf8().constData();

    // if has trailing space, was updated this cycle
    if ((index = str.find("Enabled ")) != -1)
    {
      str = str.substr(0, index);
      pear.second->setText((pear.first + ": Enabled").c_str());
    }
    // If has no trailing space, then was not updated this cycle
    else if ((index = str.find("Enabled")) != -1)
    {
      str = str.substr(0, index);
      pear.second->setText((pear.first + ": Disabled").c_str());
      pear.second->setStyleSheet(red);
    }
  }
}
}  // namespace igvc_rviz_plugins

/*
 * IMPORTANT! This macro must be filled out correctly for your panel class.
 */
PLUGINLIB_EXPORT_CLASS(igvc_rviz_plugins::sensor_panel, rviz::Panel)
