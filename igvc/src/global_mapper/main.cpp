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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map;
ros::Publisher _pointcloud_pub;
ros::Publisher _pointcloud_prob_pub;
tf::TransformListener *tf_listener;
std::set<std::string> frames_seen;
bool firstFrame;
double maxCorrDist;
int maxIter;
pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>::Ptr octree;
double searchRadius;
double octree_resolution;
double probability_thresh;

//Probabilistic mapping
//std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pointcloud_list;
std::list<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pointcloud_list;
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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr local_pc = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> local_octree(octree_resolution);

    local_octree.setInputCloud(local_pc);

    std::list<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::iterator it = pointcloud_list.begin();
    for (unsigned int i = 0; i < pointcloud_list.size(); i++)
    {
      //for (auto point : (*pointcloud_list[i]))
      for (auto point : (**it))
      {
        //G and B represent the probability, they are the same value
        //Add a point to the local octree if we haven't seen it before
        if (!local_octree.isVoxelOccupiedAtPoint(point))
        {
          point.r = 255;
          point.g = 0;
          point.b = 0;
          local_octree.addPointToCloud(point, local_pc);
        } else
        {
          //Increase probability of the point in the voxel if we see it again
          int point_idx;
          float dist;
          local_octree.approxNearestSearch(point, point_idx, dist);
          //If the probability is less than the threshold, increase the probability
          if (local_pc->points[point_idx].g < probability_thresh * 255.0) {
            local_pc->points[point_idx].g = local_pc->points[point_idx].g + (int)(255.0 / queue_window_size);
            local_pc->points[point_idx].b = local_pc->points[point_idx].b + (int)(255.0 / queue_window_size);
          } else if (!firstFrame && local_pc->points[point_idx].g >= probability_thresh * 255.0 
              && !octree->isVoxelOccupiedAtPoint(local_pc->points[point_idx]))
          {
            //Add the point to the global map if its probability is greater than or equal to the threshold
            octree->addPointToCloud(local_pc->points[point_idx], global_map);
          }
        }
      }
      it++;
    }

    (*local_pc).header.frame_id = "/odom";
    _pointcloud_prob_pub.publish(*local_pc);

    //Remove first pointcloud
    //pointcloud_list.erase(pointcloud_list.begin());
    pointcloud_list.pop_front();
    std::cout << "Map size: " << global_map->size() << std::endl;
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
  pNh.getParam("probability_thresh", probability_thresh);
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
  _pointcloud_prob_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/prob", 1);

  firstFrame = true;

  ros::spin();
}
