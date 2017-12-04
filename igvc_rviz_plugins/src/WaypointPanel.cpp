#define _USE_MATH_DEFINES

#include <igvc_rviz_plugins/WaypointPanel.h>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.h>

/*
 * Just as in the header, everything needs to happen in the igvc_rviz_plugins namespace.
 */
namespace igvc_rviz_plugins {

WaypointPanel::WaypointPanel(QWidget *parent)
  : rviz::Panel(parent) // Base class constructor
{
    // Panels are allowed to interact with NodeHandles directly just like ROS nodes.
    ros::NodeHandle handle;

    // Initialize a label for displaying some data
    QLabel *label = new QLabel("0");

    /* Initialize our subscriber to listen to the /speed topic.
    * Note the use of boost::bind, which allows us to give the callback a pointer to our UI label.
    */
    waypoint_subscriber = handle.subscribe<geometry_msgs::PointStamped>("/waypoint", 1, boost::bind(&WaypointPanel::waypoint_callback, this, _1, label));
    robot_position_subscriber = handle.subscribe<nav_msgs::Odometry>("/odometry/filtered", 1, boost::bind(&WaypointPanel::robot_position_callback, this, _1, label));

    /* Use QT layouts to add widgets to the panel.
    * Here, we're using a VBox layout, which allows us to stack all of our widgets vertically.
    */
    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(label);
    setLayout(layout);
}

void WaypointPanel::waypoint_callback(const geometry_msgs::PointStampedConstPtr &msg, QLabel *label) {
    // Create the new contents of the label based on the speed message.
    way_x = msg->point.x - robot_x;
    way_y = msg->point.y - robot_y;


    

    //bt::btMatrix3x3 m(msg->pose.pose.orientation);
    //m.getEulerYPR(yaw, pitch, roll);
    
}

void WaypointPanel::robot_position_callback(const nav_msgs::OdometryConstPtr &msg, QLabel *label) {
    double distance = sqrt(pow(way_x, 2) + pow(way_y, 2));
    double angle = atan2(way_y, way_x) * 180 / M_PI;

    robot_x = msg->pose.pose.position.x;
    robot_y = msg->pose.pose.position.y;

    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    double siny = 2.0 * (qw * qz + qx * qy);
    double cosy = 1.0 - 2.0 * (qy * qy + qz * qz);
    double yaw = atan2(siny, cosy);

    auto text = "X: " + std::to_string(way_x) + "\n" +
                "Y: " + std::to_string(way_y) + "\n" +
                "Distance: " + std::to_string(distance) + "\n" +
                "Angle: " + std::to_string(angle - yaw) + " degrees\n" +
                "Yaw: " + std::to_string(yaw) + " degrees";
    // Set the contents of the label.
    label->setText(text.c_str());

}

//void ExamplePanel::paintEvent(QPaintEvent *event)  {
//
//}

}

/*
 * IMPORTANT! This macro must be filled out correctly for your panel class.
 */
PLUGINLIB_EXPORT_CLASS( igvc_rviz_plugins::WaypointPanel, rviz::Panel)
