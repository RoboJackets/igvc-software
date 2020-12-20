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

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/statistical_outlier_removal.h>

typedef pcl::PointCloud<pcl::PointXYZ> PC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PCRGB;

struct cloud_info {
  Eigen::Vector4f centroid;
  Eigen::Vector4f min;
  Eigen::Vector4f max;
};

ClusteringNode::ClusteringNode() 
{
    private_nh_ = ros::NodeHandle("~");
    ground_filter_sub_ = private_nh_.subscribe("/ground_filter_node/ground_segmentation", 1, &ClusteringNode::clusteringCallback, this);
    clustering_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2> ("clustering_segmentation", 1);
    marker_pub_ = private_nh_.advertise<visualization_msgs::MarkerArray>("/markers",1);
};

cloud_info get_cloud_info (PCRGB::Ptr cloud) {
  cloud_info output_msg;
  pcl::compute3DCentroid (*cloud, output_msg.centroid);
  pcl::getMinMax3D (*cloud, output_msg.min, output_msg.max);
  return output_msg;
};

bool check_threshold(cloud_info cloud, int cloud_size) {
  float diff_x = cloud.max[0] - cloud.min[0];
  float diff_y = cloud.max[1] - cloud.min[1];
  float diff_z = cloud.max[2] - cloud.min[2];

  bool within_thresholds = diff_z > 0.1 && diff_x < 1.0 && diff_y < 1.0 && cloud_size > 20;
  //if (within_thresholds) ? return true : return false;
  return within_thresholds;
};

visualization_msgs::Marker mark_cluster(cloud_info cloud, int id)
{
  visualization_msgs::Marker msg;
  msg.header.frame_id = "base_link";
  msg.type = visualization_msgs::Marker::LINE_LIST;
  msg.lifetime = ros::Duration(0.1);
  msg.id = id;
  msg.action = visualization_msgs::Marker::ADD;
  msg.color.r = 1.0f;
  msg.color.g = 0.0f;
  msg.color.b = 0.0f;
  msg.color.a = 1.0f;
  msg.scale.x = 0.05;
  msg.pose.orientation.w = 1.0;
  msg.points.clear();
  
  geometry_msgs::Point p1, p2, p3, p4, p5, p6, p7, p8;
  p1.x = p2.x = p5.x = p6.x = cloud.min[0];
  p3.x = p4.x = p7.x = p8.x = cloud.max[0];
  p1.y = p3.y = p5.y = p7.y = cloud.min[1];
  p2.y = p4.y = p6.y = p8.y = cloud.max[1];
  p1.z = p2.z = p3.z = p4.z = cloud.min[2];
  p5.z = p6.z = p7.z = p8.z = cloud.max[2];
  msg.points.push_back(p1); msg.points.push_back(p2);
  msg.points.push_back(p1); msg.points.push_back(p3);
  msg.points.push_back(p2); msg.points.push_back(p4);
  msg.points.push_back(p3); msg.points.push_back(p4);
  msg.points.push_back(p1); msg.points.push_back(p5);
  msg.points.push_back(p2); msg.points.push_back(p6);
  msg.points.push_back(p3); msg.points.push_back(p7);
  msg.points.push_back(p4); msg.points.push_back(p8);
  msg.points.push_back(p5); msg.points.push_back(p6);
  msg.points.push_back(p5); msg.points.push_back(p7);
  msg.points.push_back(p6); msg.points.push_back(p8);
  msg.points.push_back(p7); msg.points.push_back(p8);

  return msg;
};

std::vector<pcl::PointIndices> euclidean_clustering(PC::Ptr input_cloud) {
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);  
  tree->setInputCloud (input_cloud);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  
  // Euclidean clustering
  ec.setClusterTolerance (0.25);
  ec.setMinClusterSize (20);
  ec.setMaxClusterSize (500);
  ec.setSearchMethod (tree);
  ec.setInputCloud (input_cloud);
  ec.extract (cluster_indices);
  return cluster_indices;
};

std::vector<pcl::PointIndices> region_growing_clustering (PC::Ptr input_cloud) {
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);  
  tree->setInputCloud (input_cloud);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (input_cloud);
  normal_estimator.setKSearch (30);
  normal_estimator.compute (*normals);

  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize (20);
  reg.setMaxClusterSize (500);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (input_cloud);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (1.0);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  return clusters;
};

void remove_outlier (PC::Ptr input_cloud) {
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (input_cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*input_cloud);
};

sensor_msgs::PointCloud2 format_output_msg (PCRGB::Ptr input_cloud) {
  sensor_msgs::PointCloud2 output_msg;
  pcl::toROSMsg(*input_cloud, output_msg);
  output_msg.header.frame_id = "base_link";
  output_msg.header.stamp = ros::Time::now();
  return output_msg;
};

void ClusteringNode::clusteringCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  PC::Ptr cloud (new PC);
  PCRGB::Ptr cloud_filtered (new PCRGB);
  PCRGB::Ptr curr_cloud (new PCRGB);
  
  pcl::fromROSMsg(*cloud_msg, *cloud);

  if (cloud->size() == 0) {
    sensor_msgs::PointCloud2 output_msg = format_output_msg (cloud_filtered);
    visualization_msgs::MarkerArray clusters_vis;
    clustering_pub_.publish (output_msg);
    marker_pub_.publish (clusters_vis);
  }
  
  else {
    visualization_msgs::MarkerArray clusters_vis;
    int counter = 0;
    std::cout << "cloud before removing outlier: " << cloud->size() << std::endl;
    remove_outlier(cloud);
    std::cout << "cloud after removing outlier: " << cloud->size() << std::endl;
    std::vector<pcl::PointIndices> cluster_indices = euclidean_clustering(cloud);
    //std::vector<pcl::PointIndices> cluster_indices = region_growing_clustering(cloud);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) 
    {
      // create a pcl object to hold the extracted cluster
      unsigned int r = (unsigned int)(rand() % 255);
      unsigned int g = (unsigned int)(rand() % 255);
      unsigned int b = (unsigned int)(rand() % 255);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      {
        pcl::PointXYZRGB p;
        p.x = cloud->points[*pit].x; p.y = cloud->points[*pit].y; p.z = cloud->points[*pit].z;
        p.r = r; p.g = g; p.b = b;
        curr_cloud->points.push_back(p);
      }
      // obtain min, max, and centroid
      cloud_info curr_cloud_info = get_cloud_info (curr_cloud);

      if (check_threshold(curr_cloud_info, curr_cloud->size())) {
        visualization_msgs::Marker marker  = mark_cluster(curr_cloud_info, counter);
        clusters_vis.markers.push_back(marker);
        *cloud_filtered += *curr_cloud;
      }
      
      counter++;
      curr_cloud->clear();
    }

    sensor_msgs::PointCloud2 output_msg = format_output_msg(cloud_filtered);
    clustering_pub_.publish (output_msg);
    marker_pub_.publish (clusters_vis);
  }
};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "clustering");
  ClusteringNode clustering = ClusteringNode();
  ros::spin();
}