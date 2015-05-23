#include "qnode.h"
#include <ros/master.h>
#include <QStringList>

using namespace std;

QNode::QNode(int argc, char** argv)
    : argc(argc),
      argv(argv)
{
}

QNode::~QNode()
{
    if(ros::isStarted())
    {
        ros::shutdown();
        ros::waitForShutdown();
    }
    wait();
}

bool QNode::init()
{
    ros::init(argc, argv,"dashboard");

    if(!ros::master::check()) {
        return false;
    }

    nh =  unique_ptr<ros::NodeHandle>(new ros::NodeHandle);

    encoder_subscriber = nh->subscribe("encoders", 1, &QNode::encoderCallback, this);

    battery_subscriber = nh->subscribe("battery", 1, &QNode::batteryCallback, this);

    log_subscriber = nh->subscribe("/rosout_agg", 1, &QNode::logCallback, this);

    map_subscriber = nh->subscribe("/map", 1, &QNode::mapCallback, this);

    estop_subscriber = nh->subscribe("/robot_enabled", 1, &QNode::estopCallback, this);

    start();
    return true;
}

void QNode::run()
{
    ros::Rate rate(1);
    while(ros::ok())
    {
        ros::spinOnce();
        ros::V_string nodes;
        if(ros::master::getNodes(nodes))
        {
            QStringList nodesList;
            for(auto node : nodes)
                nodesList.append(QString(node.c_str()));
            newNodesList(nodesList);
        }
        rate.sleep();
    }
    ros::spin();

    cout << "ROS shutting down. Closing dashboard GUI." << endl;
    emit rosShutdown();
}

void QNode::encoderCallback(const igvc_msgs::velocity_pair &msg)
{
    emit newVelocityData(fabs((msg.left_velocity + msg.right_velocity) / 2.));
}

void QNode::batteryCallback(const std_msgs::UInt8 &msg) {
    emit newBatteryLevel(msg.data);
}

void QNode::logCallback(const rosgraph_msgs::LogConstPtr &msg) {
    emit newRosoutMessage(QString((msg->name + ": " + msg->msg).c_str()));
}

void QNode::mapCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg) {
    emit newMap(msg);
}

void QNode::estopCallback(const std_msgs::BoolConstPtr &msg) {
    emit newEstopStatus(msg->data);
}
