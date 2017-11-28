#include <igvc_rviz_plugins/sensor_panel.h>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.h>
//#include <std>
//#include "rjTimer.h"

/*
 * Just as in the header, everything needs to happen in the igvc_rviz_plugins namespace.
 */
namespace igvc_rviz_plugins {


sensor_panel::sensor_panel(QWidget *parent)
  : rviz::Panel(parent) // Base class constructor
{
    // Panels are allowed to interact with NodeHandles directly just like ROS nodes.
    ros::NodeHandle nh;


    // Initialize a label for displaying some data
    label = new QLabel("IMU: Placehold\n");
	label2 = new QLabel("Lidar Sensors: Placehold\n");
	label3 = new QLabel("CamCenter: Placehold\n");
	label4 = new QLabel("CamLeft: Placehold\n");
	label5 = new QLabel("CamRight: Placehold\n");
	label6 = new QLabel("GPS: Placehold");

	/*
	*Makes timer that repeats every interval seconds
	*When timer goes off, it calls the timer method
	*/
	const int interval = 500;
	sensor_timer = new QTimer();
	sensor_timer->setSingleShot(false);//repeats
	sensor_timer->setInterval(interval);//goes off every 250 milliseconds
	//calls timer whenever sensor_timer goes off
	QObject::connect(sensor_timer, SIGNAL(timeout()),
					this, SLOT(timer()));

    /* Initialize our subscriber to listen to the /speed topic.
    * Note the use of boost::bind, which allows us to give the callback a pointer to our UI label.
    */
    imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu", 1, boost::bind(&sensor_panel::imu_callback, this, _1, label));
	lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>("/scan/pointcloud", 1, boost::bind(&sensor_panel::lidar_callback, this, _1, label2));
	camCenter_sub = nh.subscribe<sensor_msgs::Image>("/usb_cam_center/image_raw", 1, boost::bind(&sensor_panel::camCenter_callback, this, _1, label3));
	camLeft_sub = nh.subscribe<sensor_msgs::Image>("/usb_cam_left/image_raw", 1, boost::bind(&sensor_panel::camLeft_callback, this, _1, label4));
	camRight_sub = nh.subscribe<sensor_msgs::Image>("/usb_cam_right/image_raw", 1, boost::bind(&sensor_panel::camRight_callback, this, _1, label5));
	gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("/fix", 1, boost::bind(&sensor_panel::gps2_callback, this, _1, label6));

    /* Use QT layouts to add widgets to the panel.
    * Here, we're using a VBox layout, which allows us to stack all of our widgets vertically.
    */
    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(label);
	layout->addWidget(label2);
	layout->addWidget(label3);
	layout->addWidget(label4);
	layout->addWidget(label5);
	layout->addWidget(label6);
    setLayout(layout);
    timer();//inits to disabled
}

void sensor_panel::imu_callback(const sensor_msgs::ImuConstPtr &msg, QLabel *label) {
    label->setText("IMU: Enabled"/*.c_str()*/);
}

void sensor_panel::lidar_callback(const sensor_msgs::PointCloud2ConstPtr &msg, QLabel *label) {
	(*label).setText("Lidar: Enabled");
}

void sensor_panel::camCenter_callback(const sensor_msgs::ImageConstPtr &msg, QLabel *label) {
	(*label).setText("Center Camera: Enabled");
}

void sensor_panel::camLeft_callback(const sensor_msgs::ImageConstPtr &msg, QLabel *label) {
	(*label).setText("Left Camera: Enabled");
}

void sensor_panel::camRight_callback(const sensor_msgs::ImageConstPtr &msg, QLabel *label) {
	(*label).setText("Right Camera: Enabled");
}
void sensor_panel::gps2_callback(const sensor_msgs::NavSatFixConstPtr &msg, QLabel *label) {
	label->setText("GPS: Enabled");
}

/*
*Sets all of the labels to Disabled
*/
void sensor_panel::reset_labels(){
	label->setText("IMU: Disabled");
	label2->setText("Lidar: Disabled");
	label3->setText("Center Camera: Disabled");
	label4->setText("Left Camera: Disabled");
	label5->setText("Right Camera: Disabled");
	label6->setText("GPS: Disabled");

}

/*
*Sets the labels to Disabled, and then restarts the timer
*/
void sensor_panel::timer(){
	reset_labels();
	sensor_timer->start();
}

}


/*
 * IMPORTANT! This macro must be filled out correctly for your panel class.
 */
PLUGINLIB_EXPORT_CLASS( igvc_rviz_plugins::sensor_panel, rviz::Panel)
