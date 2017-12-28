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
  //Init all variables
  ros::NodeHandle nh;
  QVBoxLayout* layout = new QVBoxLayout();//layouts hold widgets
  lm = {};//LabelManager initialized

  // all labels for displaying data
  lm.labels["IMU"] = new QLabel("IMU: Placehold\n");
  lm.labels["Lidar"] = new QLabel("Lidar: Placehold\n");
  lm.labels["Center Camera"] = new QLabel("Center Camera: Placehold\n");
  lm.labels["GPS"] = new QLabel("GPS: Placehold\n");

  lm.isActive = new bool[lm.labels.size()];//make shared pointer of same size as labels


  /*
  *Makes timer that repeats every interval seconds
  *When timer goes off, it calls the timer method
  */
  lm.sensor_timer = new QTimer();
  lm.sensor_timer->setSingleShot(false);   // repeats
  lm.sensor_timer->setInterval(INTERVAL);  // goes off every 250 milliseconds
  // calls timer whenever sensor_timer goes off
  QObject::connect(lm.sensor_timer, SIGNAL(timeout()), this, SLOT(timer()));

  /* Initialize our subscriber to listen to the /speed topic.
  * Note the use of boost::bind, which allows us to give the callback a pointer to our UI label.
  */
  imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu", 1, boost::bind(&sensor_panel::imu_callback, this, _1, lm.labels["IMU"]));
  lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>("/scan/pointcloud", 1, boost::bind(&sensor_panel::lidar_callback, this, _1, lm.labels["Lidar"]));
  cam_center_sub = nh.subscribe<sensor_msgs::Image>("/usb_cam_center/image_raw", 1, boost::bind(&sensor_panel::cam_center_callback, this, _1, lm.labels["Center Camera"]));
  gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("/fix", 1, boost::bind(&sensor_panel::gps_callback, this, _1, lm.labels["GPS"]));

  
  // adds all the widgets to the layout
  for (std::pair<std::string, QLabel* > pear : lm.labels)
  {
    layout->addWidget(pear.second);
  }
  setLayout(layout);
  timer();  // inits to disabled
}

void sensor_panel::imu_callback(const sensor_msgs::ImuConstPtr &msg, QLabel* label)
{
  label->setText("IMU: Enabled");
  lm.isActive[0] = true;
}

void sensor_panel::lidar_callback(const sensor_msgs::PointCloud2ConstPtr &msg, QLabel* label)
{
  label->setText("Lidar: Enabled");
  lm.isActive[1] = true;
}

void sensor_panel::cam_center_callback(const sensor_msgs::ImageConstPtr &msg, QLabel* label)
{
  label->setText("Center Camera: Enabled");
  lm.isActive[2] = true;
}

void sensor_panel::gps_callback(const sensor_msgs::NavSatFixConstPtr &msg, QLabel* label)
{
  label->setText("GPS: Enabled");
  lm.isActive[3] = true;
}

/*
*Sets all of the activity statuses to disabled, and sets labels to disabled if already disabled
*/
void sensor_panel::reset_labels()
{
	int i = 0;//index of label's corresponding bool
    for (std::pair<std::string, QLabel* > pear : lm.labels){
		if(!lm.isActive[i]){
			pear.second->setText((pear.first+": Disabled").c_str());
		}
    else{
		  lm.isActive[i] = false;
    }
		i++;
	}
}

/*
*Sets the labels to Disabled, and then restarts the timer
*/
void sensor_panel::timer()
{
  reset_labels();
  lm.sensor_timer->start();
}
}

/*
 * IMPORTANT! This macro must be filled out correctly for your panel class.
 */
PLUGINLIB_EXPORT_CLASS(igvc_rviz_plugins::sensor_panel, rviz::Panel)
