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

  labels.insert( std::pair<int, QObject::QLabel> (0, new QLabel("IMU: Placehold\n") ) );
  labels.insert( std::pair<int, QObject::QLabel> (1, new QLabel("Lidar Sensors: Placehold\n") ) );
  labels.insert( std::pair<int, QObject::QLabel> (2, new QLabel("CamCenter: Placehold\n") ) );
  labels.insert( std::pair<int, QObject::QLabel> (3, new QLabel("GPS: Placehold") ) );

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
  imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu", 1, boost::bind(&sensor_panel::imu_callback, this, _1, labels["IMU"]));
  lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>("/scan/pointcloud", 1,
                                                     boost::bind(&sensor_panel::lidar_callback, this, _1, labels["Lidar"]));
  cam_center_sub = nh.subscribe<sensor_msgs::Image>("/usb_cam_center/image_raw", 1,
                                                   boost::bind(&sensor_panel::cam_center_callback, this, _1, labels["Center Camera"]));
  gps_sub =
      nh.subscribe<sensor_msgs::NavSatFix>("/fix", 1, boost::bind(&sensor_panel::gps2_callback, this, _1, labels["GPS"]));

  /* Use QT layouts to add widgets to the panel.
  * Here, we're using a VBox layout, which allows us to stack all of our widgets vertically.
  */
  QVBoxLayout *layout = new QVBoxLayout;

  // adds all the widgets to the layout
  for (int i=0; i<4; i++)
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

void sensor_panel::cam_center_callback(const sensor_msgs::ImageConstPtr &msg, QLabel *label)
{
  label->setText("Center Camera: Enabled");
  isActive[2] = true;
}

void sensor_panel::gps2_callback(const sensor_msgs::NavSatFixConstPtr &msg, QLabel *label)
{
  label->setText("GPS: Enabled");
  isActive[3] = true;
}

/*
*Sets all of the activity statuses to disabled, and sets labels to disabled if already disabled
*/
void sensor_panel::reset_labels()
{
  for (int i=0; i<4; i++)
  {
    if (!isActive[i])
    {
      set_label(labels[i], isActive[i]);  // set to false on second time
    }
    isActive[i] = false;  // clear all statuses
		i++;
  }
}

/*
*Sets the label at the i position to active status of b
*/
void sensor_panel::set_label(auto &s, bool b)
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

  s->setText((msg + active).c_str());
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
