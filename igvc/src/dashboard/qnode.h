#ifndef QNODE_H
#define QNODE_H

#include <QThread>
#include <ros/ros.h>
#include <memory>
#include <igvc_msgs/velocity_pair.h>
#include <std_msgs/UInt8.h>
#include <rosgraph_msgs/Log.h>
#include <QString>
#include <pcl_ros/point_cloud.h>

class QNode : public QThread
{
    Q_OBJECT
public:
    explicit QNode(int argc, char** argv);

    virtual ~QNode();

    bool init();

    void run();

    void encoderCallback(const igvc_msgs::velocity_pair& msg);

    void batteryCallback(const std_msgs::UInt8& msg);

    void logCallback(const rosgraph_msgs::LogConstPtr& msg);

    void mapCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg);

signals:
    void rosShutdown();

    void newVelocityData(float velocity);

    void newNodesList(QStringList nodes);

    void newBatteryLevel(int percent);

    void newRosoutMessage(QString message);

    void newMap(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &map);

private:
    std::unique_ptr<ros::NodeHandle> nh;

    ros::Subscriber encoder_subscriber;

    ros::Subscriber battery_subscriber;

    ros::Subscriber log_subscriber;

    ros::Subscriber map_subscriber;

    int argc;
    char** argv;

};

#endif // QNODE_H
