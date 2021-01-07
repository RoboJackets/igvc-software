#include "clustering.h"
#include "utils.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

ClusteringNode::ClusteringNode()
{
  private_nh_ = ros::NodeHandle("~");
  ground_filter_sub_ =
      private_nh_.subscribe("/ground_filter_node/ground_segmentation", 1, &ClusteringNode::clusteringCallback, this);
  clustering_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>("clustering_segmentation", 1);
  marker_pub_ = private_nh_.advertise<visualization_msgs::MarkerArray>("/markers", 1);
};

void ClusteringNode::clusteringCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  PC::Ptr cloud(new PC);
  PCRGB::Ptr cloud_filtered(new PCRGB);
  PCRGB::Ptr curr_cloud(new PCRGB);
  std::string option, frame_id;
  int cluster_min, cluster_max, kSearch, numberOfNeighbours, meanK;
  float tolerance, smoothnessThreshold, curvatureThreshold, stdDevMulThresh; 
  private_nh_.getParam("/clustering_node/option", option);
  private_nh_.getParam("/clustering_node/frame_id", frame_id);
  private_nh_.getParam("/clustering_node/outlier_filter/meanK", meanK);
  private_nh_.getParam("/clustering_node/soutlier_filter/stdDevMulThresh", stdDevMulThresh);

  if (option == "euclidean") {
    private_nh_.getParam("/clustering_node/euclidean/tolerance", tolerance);
    private_nh_.getParam("/clustering_node/euclidean/min", cluster_min);
    private_nh_.getParam("/clustering_node/euclidean/max", cluster_max);
  } 
  
  else if (option == "region_growing") {
    private_nh_.getParam("/clustering_node/region_growing/kSearch", kSearch);
    private_nh_.getParam("/clustering_node/region_growing/min", cluster_min);
    private_nh_.getParam("/clustering_node/region_growing/max", cluster_max);
    private_nh_.getParam("/clustering_node/region_growing/numberOfNeighbours", numberOfNeighbours);
    private_nh_.getParam("/clustering_node/region_growing/smoothnessThreshold", smoothnessThreshold);
    private_nh_.getParam("/clustering_node/region_growing/curvatureThreshold", curvatureThreshold);
  }

  pcl::fromROSMsg(*cloud_msg, *cloud);

  if (cloud->size() == 0)
  {
    sensor_msgs::PointCloud2 output_msg = utils::format_output_msg(cloud_filtered, frame_id);
    visualization_msgs::MarkerArray clusters_vis;
    clustering_pub_.publish(output_msg);
    marker_pub_.publish(clusters_vis);
  }

  else
  {
    visualization_msgs::MarkerArray clusters_vis;
    int counter = 0;
    utils::remove_outlier(cloud, meanK, stdDevMulThresh);
    std::vector<pcl::PointIndices> cluster_indices;

    if (option == "euclidean") {
      cluster_indices = utils::euclidean_clustering(cloud, tolerance, cluster_max, cluster_min);
    } else if (option == "region_growing") {
      cluster_indices = utils::region_growing_clustering(cloud, kSearch, cluster_min, cluster_max, numberOfNeighbours, smoothnessThreshold, curvatureThreshold);
    }

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
      // create a pcl object to hold the extracted cluster
      unsigned int r = (unsigned int)(rand() % 255);
      unsigned int g = (unsigned int)(rand() % 255);
      unsigned int b = (unsigned int)(rand() % 255);
      for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      {
        pcl::PointXYZRGB p;
        p.x = cloud->points[*pit].x;
        p.y = cloud->points[*pit].y;
        p.z = cloud->points[*pit].z;
        p.r = r;
        p.g = g;
        p.b = b;
        curr_cloud->points.push_back(p);
      }
      // obtain min, max, and centroid
      cloud_info curr_cloud_info = utils::get_cloud_info(curr_cloud);

      if (utils::check_threshold(curr_cloud_info, curr_cloud->size()))
      {
        visualization_msgs::Marker marker = utils::mark_cluster(curr_cloud_info, counter);
        clusters_vis.markers.push_back(marker);
        *cloud_filtered += *curr_cloud;
      }

      counter++;
      curr_cloud->clear();
    }

    sensor_msgs::PointCloud2 output_msg = utils::format_output_msg(cloud_filtered, frame_id);
    clustering_pub_.publish(output_msg);
    marker_pub_.publish(clusters_vis);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "clustering");
  ClusteringNode clustering = ClusteringNode();
  ros::spin();
}