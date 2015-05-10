#ifndef QNODE_H
#define QNODE_H

#include <QThread>
#include <ros/ros.h>
#include <memory>
#include <igvc_msgs/velocity_pair.h>

class QNode : public QThread
{
    Q_OBJECT
public:
    explicit QNode(int argc, char** argv);

    virtual ~QNode();

    bool init();

    void run();

    void encoderCallback(const igvc_msgs::velocity_pair& msg);

signals:
    void rosShutdown();

    void newVelocityData(float velocity);

    void newNodesList(QStringList nodes);

private:
    std::unique_ptr<ros::NodeHandle> nh;

    ros::Subscriber encoder_subscriber;

    int argc;
    char** argv;

};

#endif // QNODE_H
