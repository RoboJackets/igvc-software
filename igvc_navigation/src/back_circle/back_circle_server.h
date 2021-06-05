#ifndef SRC_BACK_CIRCLE_H
#define SRC_BACK_CIRCLE_H

#include <parameter_assertions/assertions.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <igvc_msgs/BackCircle.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

class BackCircleServer {
public:
    BackCircleServer();
    ros::NodeHandle nh_;
private:
    void waitForTransform();
    bool backCircleCallback(igvc_msgs::BackCircle::Request &req, igvc_msgs::BackCircle::Response &res);
    ros::ServiceServer back_circle_service_server_;
    tf::TransformListener tf_listener_;
};


#endif //SRC_BACK_CIRCLE_H
