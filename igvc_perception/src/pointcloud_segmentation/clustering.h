#ifndef Clustering_H
#define Clustering_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class ClusteringNode
{
public:
  ClusteringNode();
    
private:
  ros::NodeHandle private_nh_;
  ros::Subscriber ground_filter_sub_;
  ros::Publisher clustering_pub_;
  ros::Publisher marker_pub_;
  
  void clusteringCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud);
};

#endif // GroundFilterNode_H