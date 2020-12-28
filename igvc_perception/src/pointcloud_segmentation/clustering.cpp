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

  pcl::fromROSMsg(*cloud_msg, *cloud);

  if (cloud->size() == 0)
  {
    sensor_msgs::PointCloud2 output_msg = utils::format_output_msg(cloud_filtered);
    visualization_msgs::MarkerArray clusters_vis;
    clustering_pub_.publish(output_msg);
    marker_pub_.publish(clusters_vis);
  }

  else
  {
    visualization_msgs::MarkerArray clusters_vis;
    int counter = 0;
    // std::cout << "cloud before removing outlier: " << cloud->size() << std::endl;
    utils::remove_outlier(cloud);
    // std::cout << "cloud after removing outlier: " << cloud->size() << std::endl;
    std::vector<pcl::PointIndices> cluster_indices = utils::euclidean_clustering(cloud);
    // std::vector<pcl::PointIndices> cluster_indices = region_growing_clustering(cloud);

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

    sensor_msgs::PointCloud2 output_msg = utils::format_output_msg(cloud_filtered);
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