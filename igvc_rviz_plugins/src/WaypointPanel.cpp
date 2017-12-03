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
    double x = msg->point.x - robot_x;
    double y = msg->point.y - robot_y;
    double distance = sqrt(pow(x, 2) + pow(y, 2));
    double angle = atan2(y, x) * 180 / M_PI;
    double roll, pitch, yaw;
    tf::Matrix3x3 m(msg->pose.pose.orientation);
    m.getRPY(roll, pitch, yaw);
    auto text = "X: " + std::to_string(x) + "\n" +
                "Y: " + std::to_string(y) + "\n" +
                "Distance: " + std::to_string(distance) + "\n" +
                "Angle: " + std::to_string(angle - yaw) + " degrees";
    // Set the contents of the label.
    label->setText(text.c_str());
}

void WaypointPanel::robot_position_callback(const nav_msgs::OdometryConstPtr &msg, QLabel *label) {
    robot_x = msg->pose.pose.position.x;
    robot_y = msg->pose.pose.position.y;
    robot_
}

//void ExamplePanel::paintEvent(QPaintEvent *event)  {
//
//}

}

/*
 * IMPORTANT! This macro must be filled out correctly for your panel class.
 */
PLUGINLIB_EXPORT_CLASS( igvc_rviz_plugins::WaypointPanel, rviz::Panel)
