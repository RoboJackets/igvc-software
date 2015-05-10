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
