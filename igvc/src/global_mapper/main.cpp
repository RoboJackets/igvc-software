#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/octree/octree_search.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <string>

pcl::PointCloud<pcl::PointXYZ>::Ptr global_map;
ros::Publisher _pointcloud_pub;
tf::TransformListener *tf_listener;
std::set<std::string> frames_seen;
bool firstFrame;
double maxCorrDist;
int maxIter;
pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(128.0f);    //TODO: resolution?
double searchRadius;

void icp_transform(pcl::PointCloud<pcl::PointXYZ>::Ptr input)
{
  if (!firstFrame)
  {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaxCorrespondenceDistance(maxCorrDist); 
    icp.setMaximumIterations(maxIter);             
    icp.setInputSource(input);
    icp.setInputTarget(global_map);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    for (unsigned int i = 0; i < Final.size(); ++i)
    {
      pcl::PointXYZ searchPoint(Final[i].x, Final[i].y, Final[i].z);
      std::vector<int> indices;
      std::vector<float> distances;
      if (octree.radiusSearch(searchPoint, searchRadius, indices, distances, 1) == 0)
      {
        octree.addPointToCloud(searchPoint, global_map);
      }
    }
    std::cout << "Map size: " << global_map->size() << std::endl;
  }
  else if (!input->points.empty())
  {
    *global_map += *input;
    octree.setInputCloud(global_map); //Initial octree
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

  icp_transform(transformed);

  _pointcloud_pub.publish(global_map);
}

int main(int argc, char **argv)
{
  global_map = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

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
  if (topics.empty())
    ROS_WARN_STREAM("No topics specified for mapper. No map will be generated.");

  std::istringstream iss(topics);
  std::vector<std::string> tokens{ std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>() };

  for (auto topic : tokens)
  {
    ROS_INFO_STREAM("Mapper subscribing to " << topic);
    subs.push_back(nh.subscribe<pcl::PointCloud<pcl::PointXYZ> >(topic, 1, boost::bind(frame_callback, _1, topic)));
  }

  global_map->header.frame_id = "/odom";

  _pointcloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/map", 1);

  firstFrame = true;

  ros::spin();
}
