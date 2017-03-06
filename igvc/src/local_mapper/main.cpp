#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud;
ros::Publisher _pointcloud_pub;
tf::TransformListener *tf_listener;
std::set<std::string> frames_seen;

// Map of topic : cloud
std::map<std::string, pcl::PointCloud<pcl::PointXYZ> > frames;

void publish_map()
{
  map_cloud->clear();

  for (auto cloud : frames)
    *map_cloud += cloud.second;

  _pointcloud_pub.publish(map_cloud);
}

void frame_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg, const std::string &topic)
{
  pcl::PointCloud<pcl::PointXYZ> transformed;
  tf::StampedTransform transform;
  if (frames_seen.find(msg->header.frame_id) == frames_seen.end())
  {
    frames_seen.insert(msg->header.frame_id);
    tf_listener->waitForTransform("/map", msg->header.frame_id, ros::Time(0), ros::Duration(5));
  }
  tf_listener->lookupTransform("/map", msg->header.frame_id, ros::Time(0), transform);
  pcl_ros::transformPointCloud(*msg, transformed, transform);
  for (auto point : transformed)
  {
    point.z = 0;
  }

  frames[topic] = transformed;

  publish_map();
}

int main(int argc, char **argv)
{
  map_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

  ros::init(argc, argv, "local_mapper");

  ros::NodeHandle nh;
  tf_listener = new tf::TransformListener();

  std::string topics;

  std::list<ros::Subscriber> subs;

  ros::NodeHandle pNh("~");

  if (!pNh.hasParam("topics"))
    ROS_WARN_STREAM("No topics specified for mapper. No map will be generated.");

  pNh.getParam("topics", topics);

  if (topics.empty())
    ROS_WARN_STREAM("No topics specified for mapper. No map will be generated.");

  std::istringstream iss(topics);
  std::vector<std::string> tokens{ std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>() };

  for (auto topic : tokens)
  {
    ROS_INFO_STREAM("Mapper subscribing to " << topic);
    subs.push_back(nh.subscribe<pcl::PointCloud<pcl::PointXYZ> >(topic, 1, boost::bind(frame_callback, _1, topic)));
  }

  map_cloud->header.frame_id = "/map";

  _pointcloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/map", 1);

  ros::spin();
}
