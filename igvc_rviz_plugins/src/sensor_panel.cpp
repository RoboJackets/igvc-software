#include <igvc_rviz_plugins/sensor_panel.h>
#include <pluginlib/class_list_macros.h>
#include <QVBoxLayout>

/*
 * Just as in the header, everything needs to happen in the igvc_rviz_plugins namespace.
 */
namespace igvc_rviz_plugins
{
sensor_panel::sensor_panel(QWidget *parent) : rviz::Panel(parent)  // Base class constructor
{
  // Panels are allowed to interact with NodeHandles directly just like ROS nodes.
  ros::NodeHandle nh;

  // Initialize a label for displaying some data

  labels["IMU"] = new QLabel("IMU: Placehold\n");
  labels["Lidar"] = new QLabel("Lidar Sensors: Placehold\n");
  labels["Center Camera"] = new QLabel("CamCenter: Placehold\n");
  labels["Left Camera"] = new QLabel("CamLeft: Placehold\n");
  labels["Right Camera"] = new QLabel("CamRight: Placehold\n");
  labels["GPS"] = new QLabel("GPS: Placehold");

  /*
  *Makes timer that repeats every interval seconds
  *When timer goes off, it calls the timer method
  */
  sensor_timer = new QTimer();
  sensor_timer->setSingleShot(false);   // repeats
  sensor_timer->setInterval(INTERVAL);  // goes off every 250 milliseconds
  // calls timer whenever sensor_timer goes off
  QObject::connect(sensor_timer, SIGNAL(timeout()), this, SLOT(timer()));

  /* Initialize our subscriber to listen to the /speed topic.
  * Note the use of boost::bind, which allows us to give the callback a pointer to our UI label.
  */
  imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu", 1, boost::bind(&sensor_panel::imu_callback, this, _1, labels[0]));
  lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>("/scan/pointcloud", 1,
                                                     boost::bind(&sensor_panel::lidar_callback, this, _1, labels[1]));
  camCenter_sub = nh.subscribe<sensor_msgs::Image>("/usb_cam_center/image_raw", 1,
                                                   boost::bind(&sensor_panel::camCenter_callback, this, _1, labels[2]));
  camLeft_sub = nh.subscribe<sensor_msgs::Image>("/usb_cam_left/image_raw", 1,
                                                 boost::bind(&sensor_panel::camLeft_callback, this, _1, labels[3]));
  camRight_sub = nh.subscribe<sensor_msgs::Image>("/usb_cam_right/image_raw", 1,
                                                  boost::bind(&sensor_panel::camRight_callback, this, _1, labels[4]));
  gps_sub =
      nh.subscribe<sensor_msgs::NavSatFix>("/fix", 1, boost::bind(&sensor_panel::gps2_callback, this, _1, labels[5]));

  /* Use QT layouts to add widgets to the panel.
  * Here, we're using a VBox layout, which allows us to stack all of our widgets vertically.
  */
  QVBoxLayout *layout = new QVBoxLayout;

  // adds all the widgets to the layout
  for (int i = 0; i < NUMSENSORS; i++)
  {
    layout->addWidget(labels[i]);
  }
  setLayout(layout);
  timer();  // inits to disabled
}

void sensor_panel::imu_callback(const sensor_msgs::ImuConstPtr &msg, QLabel *label)
{
  label->setText("IMU: Enabled");
  isActive[0] = true;
}

void sensor_panel::lidar_callback(const sensor_msgs::PointCloud2ConstPtr &msg, QLabel *label)
{
  label->setText("Lidar: Enabled");
  isActive[1] = true;
}

void sensor_panel::cam_Center_callback(const sensor_msgs::ImageConstPtr &msg, QLabel *label)
{
  label->setText("Center Camera: Enabled");
  isActive[2] = true;
}

void sensor_panel::cam_Left_callback(const sensor_msgs::ImageConstPtr &msg, QLabel *label)
{
  label->setText("Left Camera: Enabled");
  isActive[3] = true;
}

void sensor_panel::cam_Right_callback(const sensor_msgs::ImageConstPtr &msg, QLabel *label)
{
  label->setText("Right Camera: Enabled");
  isActive[4] = true;
}
void sensor_panel::gps2_callback(const sensor_msgs::NavSatFixConstPtr &msg, QLabel *label)
{
  label->setText("GPS: Enabled");
  isActive[5] = true;
}

/*
*Sets all of the activity statuses to disabled, and sets labels to diabled if already disabled
*/
void sensor_panel::reset_labels()
{
  for (bool b : isActive)
  {
    if (!b)
    {
      set_label(i, b);  // set to false on second time
    }
    b = false;  // clear all statuses
  }
}

/*
*Sets the label at the i position to active status of b
*/
void sensor_panel::set_label(std::string s, bool b)
{
  std::string active, msg;
  if (b)
  {
    active = "Enabled";
  }
  else
  {
    active = "Disabled";
  }
  msg = msg +": ";

  labels[i]->setText((msg + active).c_str());
}

/*
*Sets the labels to Disabled, and then restarts the timer
*/
void sensor_panel::timer()
{
  reset_labels();
  sensor_timer->start();
}
}

/*
 * IMPORTANT! This macro must be filled out correctly for your panel class.
 */
PLUGINLIB_EXPORT_CLASS(igvc_rviz_plugins::sensor_panel, rviz::Panel)
