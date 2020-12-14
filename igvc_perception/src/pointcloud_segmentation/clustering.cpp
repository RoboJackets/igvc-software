#include "clustering.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>

typedef pcl::PointCloud<pcl::PointXYZ> PC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PCRGB;

ClusteringNode::ClusteringNode() 
{
    private_nh_ = ros::NodeHandle("~");
    ground_filter_sub_ = private_nh_.subscribe("/ground_filter_node/ground_segmentation", 1, &ClusteringNode::clusteringCallback, this);
    clustering_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2> ("clustering_segmentation", 1);
    marker_pub_ = private_nh_.advertise<visualization_msgs::MarkerArray>("/markers",1);
};

visualization_msgs::Marker mark_cluster(PCRGB::Ptr cloud, int id)
{
  Eigen::Vector4f centroid;
  Eigen::Vector4f min;
  Eigen::Vector4f max;

  pcl::compute3DCentroid (*cloud, centroid);
  pcl::getMinMax3D (*cloud, min, max);
  float diff_x = max[0] - min[0];
  float diff_y = max[1] - min[1];
  float diff_z = max[2] - min[2];
  bool within_thresholds = diff_z > 0.1 && diff_x < 0.9 && diff_y < 0.9 && cloud->size()>20;
  
  /*std::cout << "start-----------------" << std::endl;
  std::cout << "z: " << max[2] - min[2] << std::endl;
  std::cout << "x: " << max[0] - min[0] << std::endl;
  std::cout << "y: " << max[1] - min[1] << std::endl;
  std::cout << "end-------------------" << std::endl;*/
  
  visualization_msgs::Marker msg;
  msg.header.frame_id = "base_link";
  msg.type = visualization_msgs::Marker::LINE_LIST;
  msg.lifetime = ros::Duration();
  msg.id = id;
  msg.action = visualization_msgs::Marker::ADD;
  msg.color.r = 1.0f;
  msg.color.g = 0.0f;
  msg.color.b = 0.0f;
  msg.color.a = 1.0f;
  msg.scale.x = 0.05;
  msg.pose.orientation.w = 1.0;
  msg.points.clear();
  
  if (within_thresholds) {
    geometry_msgs::Point p1,p2,p3,p4,p5,p6,p7,p8;
    p1.x = p2.x = p5.x = p6.x = min[0];
    p3.x = p4.x = p7.x = p8.x = max[0];
    p1.y = p3.y = p5.y = p7.y = min[1];
    p2.y = p4.y = p6.y = p8.y = max[1];
    p1.z = p2.z = p3.z = p4.z = min[2];
    p5.z = p6.z = p7.z = p8.z = max[2];
    msg.points.push_back(p1);msg.points.push_back(p2);
    msg.points.push_back(p1);msg.points.push_back(p3);
    msg.points.push_back(p2);msg.points.push_back(p4);
    msg.points.push_back(p3);msg.points.push_back(p4);
    msg.points.push_back(p1);msg.points.push_back(p5);
    msg.points.push_back(p2);msg.points.push_back(p6);
    msg.points.push_back(p3);msg.points.push_back(p7);
    msg.points.push_back(p4);msg.points.push_back(p8);
    msg.points.push_back(p5);msg.points.push_back(p6);
    msg.points.push_back(p5);msg.points.push_back(p7);
    msg.points.push_back(p6);msg.points.push_back(p8);
    msg.points.push_back(p7);msg.points.push_back(p8);
  }

  return msg;
};

void ClusteringNode::clusteringCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  PC::Ptr cloud (new PC);
  PCRGB::Ptr cloud_filtered (new PCRGB);
  PCRGB::Ptr curr_cloud (new PCRGB);
  pcl::fromROSMsg(*cloud_msg, *cloud);
  
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);  
  tree->setInputCloud (cloud);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  
  ec.setClusterTolerance (0.25);
  ec.setMinClusterSize (20);
  ec.setMaxClusterSize (500);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

  // declare the output variable instances
  sensor_msgs::PointCloud2 output_msg;
  visualization_msgs::MarkerArray clusters_vis;
  int counter = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) 
  {
    // create a pcl object to hold the extracted cluster
    //PCRGB::Ptr cluster (new PCRGB);
    unsigned int r = (unsigned int)(rand() % 255);
    unsigned int g = (unsigned int)(rand() % 255);
    unsigned int b = (unsigned int)(rand() % 255);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      pcl::PointXYZRGB p;
      p.x = cloud->points[*pit].x; p.y = cloud->points[*pit].y; p.z = cloud->points[*pit].z;
      p.r = r; p.g = g; p.b = b;
      curr_cloud->points.push_back(p);
      cloud_filtered->points.push_back(p);
    }

    //std::cout << curr_cloud->points.size() << " cluster points: " << counter << std::endl;
    visualization_msgs::Marker marker  = mark_cluster(curr_cloud, counter);

    clusters_vis.markers.push_back(marker);
    
    counter++;
    curr_cloud->clear();
  }
  
  std::cout << " number of clusters: " << counter << std::endl;
  pcl::toROSMsg(*cloud_filtered, output_msg);
  output_msg.header.frame_id = "base_link";
  output_msg.header.stamp = ros::Time::now();
  clustering_pub_.publish (output_msg);
  marker_pub_.publish (clusters_vis);
};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "clustering");
  ClusteringNode clustering = ClusteringNode();
  ros::spin();
}