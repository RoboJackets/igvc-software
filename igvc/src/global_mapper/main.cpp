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
#include <stdlib.h>
#include <string>
#include <queue>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map;
ros::Publisher _pointcloud_pub;
ros::Publisher _pointcloud_incremental_pub;
tf::TransformListener *tf_listener;
std::set<std::string> frames_seen;
bool firstFrame;
double maxCorrDist;
int maxIter;
pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>::Ptr octree;
double searchRadius;
double octree_resolution;

//Probabilistic mapping
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pointcloud_list;
int queue_window_size;

void icp_transform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input)
{
  if (pointcloud_list.size() < queue_window_size)
  {
    pointcloud_list.push_back(input);
  } 
  if (pointcloud_list.size() == queue_window_size)
  {
    //Probabilistic calculations here
    pcl::PointCloud<pcl::PointXYZRGB> local_pc;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> local_octree(octree_resolution);

    //Remove first pointcloud
    pointcloud_list.erase(pointcloud_list.begin());
  }
  if (!firstFrame)
  {
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setMaxCorrespondenceDistance(maxCorrDist);
    icp.setMaximumIterations(maxIter);
    icp.setInputSource(input);
    icp.setInputTarget(global_map);
    pcl::PointCloud<pcl::PointXYZRGB> Final;
    pcl::PointCloud<pcl::PointXYZRGB> new_points;
    icp.align(Final);
    for (unsigned int i = 0; i < Final.size(); ++i)
    {
      pcl::PointXYZRGB searchPoint(255, 0, 0);
      searchPoint.x = Final[i].x;
      searchPoint.y = Final[i].y;
      searchPoint.z = Final[i].z;

      if (!octree->isVoxelOccupiedAtPoint(searchPoint)) {
        octree->addPointToCloud(searchPoint, global_map);
        new_points.push_back(searchPoint);
      }
    }
    new_points.header.frame_id = "/odom";
    _pointcloud_incremental_pub.publish(new_points);
    std::cout << "Map size: " << global_map->size() << std::endl;
  }
  else if (!input->points.empty())
  { 
    octree->setInputCloud(global_map);
    pcl::PointCloud<pcl::PointXYZRGB> initial = *input;
    for (unsigned int i = 0; i < initial.size(); ++i)
    {
      pcl::PointXYZRGB searchPoint(255, 0, 0);
      searchPoint.x = initial[i].x;
      searchPoint.y = initial[i].y;
      searchPoint.z = initial[i].z;

      if (!octree->isVoxelOccupiedAtPoint(searchPoint)) {
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
  if (frames_seen.find(msg->header.frame_id) == frames_seen.end())
  {
    frames_seen.insert(msg->header.frame_id);
    tf_listener->waitForTransform("/odom", msg->header.frame_id, ros::Time(0), ros::Duration(5));
  }
  tf_listener->lookupTransform("/odom", msg->header.frame_id, ros::Time(0), transform);
  pcl_ros::transformPointCloud(*msg, *transformed, transform);
  for (auto point : *transformed)
  {
    point.z = 0;
  }

  //Convert PointXYZ to PointXYZRGB
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_rgb = 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::copyPointCloud(*transformed, *transformed_rgb);
  icp_transform(transformed_rgb );

  _pointcloud_pub.publish(global_map);
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
  pNh.getParam("queue_window_size", queue_window_size);
  if (topics.empty())
    ROS_WARN_STREAM("No topics specified for mapper. No map will be generated.");

  octree = pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>::Ptr(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(octree_resolution));
  std::istringstream iss(topics);
  std::vector<std::string> tokens{ std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>() };

  for (auto topic : tokens)
  {
    ROS_INFO_STREAM("Mapper subscribing to " << topic);
    subs.push_back(nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>(topic, 1, boost::bind(frame_callback, _1, topic)));
  }

  global_map->header.frame_id = "/odom";

  _pointcloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/map", 1);
  _pointcloud_incremental_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/map/incremental", 1);

  firstFrame = true;

  ros::spin();
}
