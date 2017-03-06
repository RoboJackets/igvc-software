#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

// #include <pcl/filters/voxel_grid.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud;
ros::Publisher _pointcloud_pub;
tf::TransformListener *tf_listener;
std::set<std::string> frames_seen;

double distance(pcl::PointXYZ a, pcl::PointXYZ b)
{
  return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
}

void filterOutDuplicates(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  std::vector<int> indecesToRemove;

  for (unsigned int i = 0; i < cloud->size(); i++)
  {
    for (unsigned int j = i + 1; j < cloud->size(); j++)
    {
      if (distance(cloud->points[i], cloud->points[j]) < 0.10)
      {
        indecesToRemove.push_back(i);
        i++;
        break;
      }
    }
  }

  sort(indecesToRemove.begin(), indecesToRemove.end(), std::greater<int>());
  for (auto idx : indecesToRemove)
    cloud->erase(cloud->points.begin() + idx);

  /*
   * This code suddenly started erasing all the points in the
   *cloud during comp, so we wrote the woefully inefficient code above.
   */
  /*VoxelGrid<PointXYZ> filter;
  filter.setInputCloud(cloud);

  auto minx = cloud->at(0).x;
  auto maxx =  cloud->at(0).x;
  auto miny = cloud->at(0).y;
  auto maxy = cloud->at(0).y;

  for(auto point : *cloud)
  {
      minx = min(minx, point.x);
      maxx = max(maxx, point.x);
      miny = min(miny, point.y);
      maxy = max(maxy, point.y);
  }
  auto xrange = maxx - minx;
  auto yrange = maxy - miny;

  auto size = max((xrange*yrange)/ (INT32_MAX / 2.0), 0.05);

  ROS_INFO_STREAM("Range: " << (xrange * yrange ));
  ROS_INFO_STREAM("Size of leaf: " << size);
  filter.setLeafSize(size, size, 2.0);

  ROS_INFO_STREAM("cloud    " <<  cloud->size());

  PointCloud<PointXYZ> newcloud;
  filter.filter(newcloud);

  ROS_INFO_STREAM("divisions: " << filter.getNrDivisions());

  map_cloud->swap(newcloud);*/
}

void frame_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg)
{
  ROS_INFO("NODECALLBACK");
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

  *map_cloud += transformed;

  ROS_INFO_STREAM("cloud size before call: " << map_cloud->size());
  filterOutDuplicates(map_cloud);
  ROS_INFO_STREAM("cloud size after call: " << map_cloud->size());

  _pointcloud_pub.publish(map_cloud);
}

int main(int argc, char **argv)
{
  map_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

  ros::init(argc, argv, "mapper");

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
    subs.push_back(nh.subscribe(topic, 1, frame_callback));
  }

  map_cloud->header.frame_id = "/map";

  _pointcloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/map", 1);

  ros::spin();
}
