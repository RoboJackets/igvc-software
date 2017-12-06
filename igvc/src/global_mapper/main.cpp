#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <stdlib.h>
#include <string>
#include <unordered_map>
#include <utility>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map;
ros::Publisher _pointcloud_pub;
ros::Publisher _pointcloud_prob_pub;
ros::Publisher _pointcloud_size_pub;
tf::TransformListener *tf_listener;
bool firstFrame;
double maxCorrDist;
int maxIter;
pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>::Ptr octree;
double searchRadius;
double octree_resolution;
double probability_thresh;

std::unordered_map<std::string, std::pair<std::list<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> *, int>> topic_pc_map;

std::vector<int> queue_window_sizes;
bool use_icp;

void icp_transform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, const std::string &topic)
{
  // Retrieve pointcloud list pointer from unordered map
  auto map_it = topic_pc_map.find(topic);
  std::list<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> *pointcloud_list = map_it->second.first;

  // Temp
  int queue_window_size = map_it->second.second;

  if (pointcloud_list->size() < static_cast<unsigned int>(queue_window_size))
  {
    // icp
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final =
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    if (!firstFrame && global_map->size() > 0 && use_icp)
    {
      pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
      icp.setMaxCorrespondenceDistance(maxCorrDist);
      icp.setMaximumIterations(maxIter);
      icp.setInputSource(input);
      icp.setInputTarget(global_map);
      pcl::PointCloud<pcl::PointXYZRGB> new_points;
      icp.align(*Final);
      pointcloud_list->push_back(Final);
    }
    else
    {
      pointcloud_list->push_back(input);
    }
  }
  if (pointcloud_list->size() == static_cast<unsigned int>(queue_window_size))
  {
    // Probabilistic calculations here
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr local_pc =
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> local_octree(octree_resolution);

    local_octree.setInputCloud(local_pc);

    for (auto it = pointcloud_list->begin(); it != pointcloud_list->end(); it++)
    {
      for (auto point : (**it))
      {
        // G and B represent the probability, they are the same value
        // Add a point to the local octree if we haven't seen it before
        if (!local_octree.isVoxelOccupiedAtPoint(point))
        {
          point.r = 255;
          point.g = 0;
          point.b = 0;
          local_octree.addPointToCloud(point, local_pc);
        }
        else
        {
          // Increase probability of the point in the voxel if we see it again
          int point_idx;
          float dist;
          local_octree.approxNearestSearch(point, point_idx, dist);
          // If the probability is less than the threshold, increase the probability
          local_pc->points[point_idx].g = local_pc->points[point_idx].g + (int)(255.0 / queue_window_size);
          local_pc->points[point_idx].b = local_pc->points[point_idx].b + (int)(255.0 / queue_window_size);
          if (local_pc->points[point_idx].g >= probability_thresh * 255.0 &&
              !octree->isVoxelOccupiedAtPoint(local_pc->points[point_idx]))
          {
            // Add the point to the global map if its probability is greater than or equal to the threshold
            octree->addPointToCloud(local_pc->points[point_idx], global_map);
          }
        }
      }
    }

    (*local_pc).header.frame_id = "/odom";
    _pointcloud_prob_pub.publish(*local_pc);

    // Remove first pointcloud
    pointcloud_list->pop_front();

    // Publish map size
    std_msgs::Int64 size_msg;
    size_msg.data = global_map->size();
    _pointcloud_size_pub.publish(size_msg);
  }

  if (firstFrame && !input->points.empty())
  {
    octree->setInputCloud(global_map);
    for (unsigned int i = 0; i < input->size(); ++i)
    {
      pcl::PointXYZRGB searchPoint(255, 0, 0);
      searchPoint.x = input->points[i].x;
      searchPoint.y = input->points[i].y;
      searchPoint.z = input->points[i].z;

      if (!octree->isVoxelOccupiedAtPoint(searchPoint))
      {
        octree->addPointToCloud(searchPoint, global_map);
      }
    }
    firstFrame = false;
  }
}

void frame_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg, const std::string &topic)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  tf::StampedTransform transform;
  if (tf_listener->waitForTransform("/odom", msg->header.frame_id, ros::Time(0), ros::Duration(5.0)))
  {
    tf_listener->lookupTransform("/odom", msg->header.frame_id, ros::Time(0), transform);
    pcl_ros::transformPointCloud(*msg, *transformed, transform);
    for (auto point : *transformed)
    {
      point.z = 0;
    }

    // Convert PointXYZ to PointXYZRGB
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_rgb =
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::copyPointCloud(*transformed, *transformed_rgb);
    icp_transform(transformed_rgb, topic);

    _pointcloud_pub.publish(global_map);
  }
  else
  {
    ROS_WARN_STREAM("Could not find transform to odom");
  }
}

int main(int argc, char **argv)
{
  global_map = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

  ros::init(argc, argv, "global_mapper");
  ros::NodeHandle nh;
  tf_listener = new tf::TransformListener();

  std::string topics;
  std::list<ros::Subscriber> subs;

  ros::NodeHandle pNh("~");
  if (!pNh.hasParam("topics"))
    ROS_WARN_STREAM("No topics specified for mapper. No map will be generated.");

  pNh.getParam("topics", topics);
  pNh.getParam("max_correspondence_distance", maxCorrDist);
  pNh.getParam("max_iterations", maxIter);
  pNh.getParam("search_radius", searchRadius);
  pNh.getParam("octree_resolution", octree_resolution);
  pNh.getParam("probability_thresh", probability_thresh);
  pNh.getParam("queue_window_sizes", queue_window_sizes);
  pNh.getParam("use_icp", use_icp);
  if (topics.empty())
    ROS_WARN_STREAM("No topics specified for mapper. No map will be generated.");

  octree = pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>::Ptr(
      new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(octree_resolution));
  std::istringstream iss(topics);
  std::vector<std::string> tokens{ std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>() };

  if (tokens.size() != queue_window_sizes.size())
  {
    std::cout << tokens.size() << " " << queue_window_sizes.size() << std::endl;
    ROS_ERROR_STREAM("Queue window size does not match topic list size");
    return 0;
  }

  int count = 0;
  for (auto topic : tokens)
  {
    ROS_INFO_STREAM("Mapper subscribing to " << topic);
    subs.push_back(nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>(topic, 1, boost::bind(frame_callback, _1, topic)));
    topic_pc_map.insert(
        { topic, std::make_pair(new std::list<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>, queue_window_sizes[count]) });
    count++;
  }

  global_map->header.frame_id = "/odom";

  _pointcloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/map", 1);
  _pointcloud_prob_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/prob", 1);
  _pointcloud_size_pub = nh.advertise<std_msgs::Int64>("/map_size", 1);

  firstFrame = true;

  ros::spin();
}
