#ifndef GroundFilter_H
#define GroundFilter_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class GroundFilterNode
{
public:
  GroundFilterNode();
    
private:
  ros::NodeHandle private_nh_;
  ros::Subscriber raw_pts_sub_;
  ros::Publisher ground_filter_pub_;
  
  void groundFilterCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud);
};

#endif // GroundFilterNode_H