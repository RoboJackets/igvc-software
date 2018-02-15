#define _USE_MATH_DEFINES

#include <igvc_rviz_plugins/WaypointPanel.h>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.h>

namespace igvc_rviz_plugins {

WaypointPanel::WaypointPanel(QWidget *parent)
  : rviz::Panel(parent)
{
    ros::NodeHandle handle;

    QLabel *label = new QLabel("0");

    waypoint_subscriber = handle.subscribe<geometry_msgs::PointStamped>("/waypoint", 1, boost::bind(&WaypointPanel::waypoint_callback, this, _1, label));
    robot_position_subscriber = handle.subscribe<nav_msgs::Odometry>("/odometry/filtered", 1, boost::bind(&WaypointPanel::robot_position_callback, this, _1, label));

    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(label);
    setLayout(layout);
}

void WaypointPanel::waypoint_callback(const geometry_msgs::PointStampedConstPtr &msg, QLabel *label) {
    way_x = msg->point.x - robot_x;
    way_y = msg->point.y - robot_y;
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

    double yaw = atan2(siny, cosy) * 180 / M_PI;

    auto text = "X: " + std::to_string(way_x) + "\n" +
                "Y: " + std::to_string(way_y) + "\n" +
                "Distance: " + std::to_string(distance) + "\n" +
                "Angle: " + std::to_string(angle - yaw) + " degrees\n";
    
    label->setText(text.c_str());

}

}

PLUGINLIB_EXPORT_CLASS( igvc_rviz_plugins::WaypointPanel, rviz::Panel)
