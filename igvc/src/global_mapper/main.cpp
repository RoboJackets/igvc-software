#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <stdlib.h>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

struct WatchedPointcloud
{
  std::string topic_name;
  std::list<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> *window;
  int window_size;
  double probability_threshold;
};

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

std::unordered_map<std::string, std::shared_ptr<WatchedPointcloud>> topic_pc_map;

std::vector<int> queue_window_sizes;
std::vector<double> probability_thresholds;
bool use_icp;

void filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, const std::string &topic)
{
  // Retrieve pointcloud list pointer from unordered map
  auto current_watched = topic_pc_map.find(topic)->second;
  std::list<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> *pointcloud_list = current_watched->window;

  if (pointcloud_list->size() < static_cast<unsigned int>(current_watched->window_size))
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
  if (pointcloud_list->size() == static_cast<unsigned int>(current_watched->window_size))
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
          local_pc->points[point_idx].g = local_pc->points[point_idx].g + (int)(255.0 / current_watched->window_size);
          local_pc->points[point_idx].b = local_pc->points[point_idx].b + (int)(255.0 / current_watched->window_size);
          if (local_pc->points[point_idx].g >= current_watched->probability_threshold * 255.0 &&
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
  firstFrame = false;
}

void frame_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg, const std::string &topic)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  tf::StampedTransform transform;
  ros::Time time = pcl_conversions::fromPCL(msg->header.stamp);
  if (tf_listener->waitForTransform("/odom", msg->header.frame_id, time, ros::Duration(5.0)))
  {
    tf_listener->lookupTransform("/odom", msg->header.frame_id, time, transform);
    pcl_ros::transformPointCloud(*msg, *transformed, transform);
    for (auto point : *transformed)
    {
      point.z = 0;
    }

    // Convert PointXYZ to PointXYZRGB
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_rgb =
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::copyPointCloud(*transformed, *transformed_rgb);
    filter(transformed_rgb, topic);

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
  pNh.getParam("probability_thresholds", probability_thresholds);
  pNh.getParam("queue_window_sizes", queue_window_sizes);
  pNh.getParam("use_icp", use_icp);
  if (topics.empty())
    ROS_WARN_STREAM("No topics specified for mapper. No map will be generated.");

  octree = pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>::Ptr(
      new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(octree_resolution));
  octree->setInputCloud(global_map);
  std::istringstream iss(topics);
  std::vector<std::string> tokens{ std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>() };

  if (tokens.size() != queue_window_sizes.size() || tokens.size() != probability_thresholds.size())
  {
    std::cout << tokens.size() << " " << queue_window_sizes.size() << " " << probability_thresholds.size() << std::endl;
    ROS_ERROR_STREAM("Queue window size, number of topics, and probability thresholds size does not match");
    return 0;
  }

  int count = 0;
  for (std::string topic : tokens)
  {
    ROS_INFO_STREAM("Mapper subscribing to " << topic);
    subs.push_back(nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>(topic, 1, boost::bind(frame_callback, _1, topic)));
    std::shared_ptr<WatchedPointcloud> current = std::make_shared<WatchedPointcloud>();
    current->topic_name = topic;
    current->window = new std::list<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>;
    current->window_size = queue_window_sizes[count];
    current->probability_threshold = probability_thresholds[count];
    topic_pc_map.insert({ topic, current });
    count++;
  }

  global_map->header.frame_id = "/odom";

  _pointcloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/map", 1);
  _pointcloud_prob_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/prob", 1);
  _pointcloud_size_pub = nh.advertise<std_msgs::Int64>("/map_size", 1);

  firstFrame = true;

  ros::spin();
}
